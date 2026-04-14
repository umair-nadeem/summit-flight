#pragma once

#include "boundaries/SharedData.hpp"
#include "imu/ImuData.hpp"
#include "imu/ImuStatus.hpp"
#include "imu_sensor/ImuSensorStatus.hpp"
#include "imu_sensor/RawImuSensorData.hpp"
#include "interfaces/IClockSource.hpp"

namespace imu
{

template <interfaces::IClockSource ClockSource, typename Logger>
class Imu
{
   using ImuDataPublisher   = boundaries::SharedData<imu::ImuData>;
   using ImuStatusPublisher = boundaries::SharedData<imu::ImuStatus>;
   using Vec3               = math::Vec3f;

public:
   explicit Imu(ImuDataPublisher&   imu_data_publisher,
                ImuStatusPublisher& imu_health_publisher,
                Logger&             logger,
                const uint16_t      num_calibration_samples,
                const float         gyro_tolerance_radps,
                const float         accel_tolerance_mps2,
                const bool          front_left_up_frame)
       : m_imu_data_publisher{imu_data_publisher},
         m_imu_health_publisher{imu_health_publisher},
         m_logger{logger},
         m_num_calibration_samples{num_calibration_samples},
         m_gyro_tolerance_radps{gyro_tolerance_radps},
         m_accel_tolerance_mps2{accel_tolerance_mps2},
         m_front_left_up_frame{front_left_up_frame}

   {
      m_logger.enable();
   }

   void execute(const imu_sensor::RawImuSensorData& sensor_data, const imu_sensor::ImuSensorStatus& sensor_status)
   {
      m_clock_ms = ClockSource::now_ms();

      if (!sensor_available(sensor_status))
      {
         if (m_calibration.state != Calibration::State::stopped)
         {
            m_calibration.state                   = Calibration::State::stopped;
            m_imu_status.calibration_failure      = true;
            m_imu_data.status.calibration_ongoing = false;
            m_logger.print("sensor error - stopping calibration");
         }
         reset_data();
         publish_health();
         return;
      }

      imu_sensor::RawImuSensorData raw_sensor_data{sensor_data};
      if (is_stale(raw_sensor_data))
      {
         publish_health();
         return;
      }

      if (m_calibration.state == Calibration::State::stopped)
      {
         process_raw_sensor_data(raw_sensor_data);
      }
      else
      {
         process_calibration(raw_sensor_data);
         reset_data();
      }

      publish_data();
      publish_health();
      m_previous_raw_sensor_data = raw_sensor_data;
   }

   void start_calibration()
   {
      if (m_calibration.state == Calibration::State::stopped)
      {
         m_calibration.state                   = Calibration::State::started;
         m_imu_data.status.calibration_ongoing = true;
         m_imu_status.calibration_done         = false;
         m_logger.print("starting calibration");
         publish_health();
      }
   }

private:
   void process_raw_sensor_data(imu_sensor::RawImuSensorData& raw_data)
   {
      adjust_bias(raw_data);

      perform_alignment(raw_data);

      m_imu_data.accel_mps2    = raw_data.accel_mps2;
      m_imu_data.gyro_radps    = raw_data.gyro_radps;
      m_imu_data.temperature_c = raw_data.temperature_c;
   }

   bool sensor_available(const imu_sensor::ImuSensorStatus& sensor_status)
   {
      m_imu_status.available = !sensor_status.fault && sensor_status.validation_ok && sensor_status.config_ok;
      return m_imu_status.available;
   }

   void publish_data()
   {
      m_imu_data_publisher.update_latest(m_imu_data, m_clock_ms);
   }

   void publish_health()
   {
      m_imu_health_publisher.update_latest(m_imu_status, m_clock_ms);
   }

   void adjust_bias(auto& raw_data) const
   {
      if (m_imu_status.calibration_done && (m_calibration.state == Calibration::State::stopped))
      {
         raw_data.accel_mps2 = raw_data.accel_mps2 - m_bias.accel;
         raw_data.gyro_radps = raw_data.gyro_radps - m_bias.gyro;
      }
   }

   void perform_alignment(auto& raw_data) const
   {
      if (m_front_left_up_frame)
      {
         // convert sensor axes from FLU body frame to FRD
         raw_data.accel_mps2 = convert_to_front_right_down_frame(raw_data.accel_mps2);
         raw_data.gyro_radps = convert_to_front_right_down_frame(raw_data.gyro_radps);
      }
   }

   void reset_data()
   {
      m_imu_data.accel_mps2.reset();
      m_imu_data.gyro_radps.reset();
      m_imu_data.temperature_c.reset();
   }

   void process_calibration(const auto& raw_data)
   {
      switch (m_calibration.state)
      {
         case Calibration::State::started:

            m_calibration.reset();
            m_calibration.state = Calibration::State::running;
            break;

         case Calibration::State::running:

            add_calibration_sample(raw_data.accel_mps2, raw_data.gyro_radps);
            if (all_samples_collected())
            {
               compute_stats();
               m_calibration.state = Calibration::State::evaluating;
            }
            break;

         case Calibration::State::evaluating:

            if (is_calibration_valid())
            {
               calculate_bias();
               reset_calibration_failure_errors();
               m_imu_status.calibration_done    = true;
               m_imu_status.calibration_failure = false;
               m_logger.print("calibration done");
            }
            else
            {
               set_calibration_failure_error();
               m_imu_status.calibration_failure = true;
               m_logger.print("calibration failed");
            }
            m_imu_data.status.calibration_ongoing = false;
            m_calibration.state                   = Calibration::State::stopped;
            break;

         case Calibration::State::stopped:
            break;

         default:
            error::stop_operation();
            break;
      }
   }

   // assumes Front-Left-Up and converts to Front-Right-Down body frame
   static constexpr auto convert_to_front_right_down_frame(const math::Vec3f& vector3_flu)
   {
      return math::Vec3f{vector3_flu[0], -vector3_flu[1], -vector3_flu[2]};
   }

   void add_calibration_sample(const Vec3& accel, const Vec3& gyro)
   {
      m_calibration.sum_accel += accel;
      m_calibration.sum_gyro += gyro;

      m_calibration.sum_sq_accel += accel.emul(accel);
      m_calibration.sum_sq_gyro += gyro.emul(gyro);

      m_calibration.sample_counter++;
   }

   void compute_stats()
   {
      const float n = static_cast<float>(m_calibration.sample_counter);

      m_calibration.mean_accel = m_calibration.sum_accel / n;
      m_calibration.mean_gyro  = m_calibration.sum_gyro / n;

      const Vec3 variance_accel = (m_calibration.sum_sq_accel / n) - m_calibration.mean_accel.emul(m_calibration.mean_accel);
      const Vec3 variance_gyro  = (m_calibration.sum_sq_gyro / n) - m_calibration.mean_gyro.emul(m_calibration.mean_gyro);

      const float std_accel_x = sqrtf(variance_accel[0]);
      const float std_accel_y = sqrtf(variance_accel[1]);
      const float std_accel_z = sqrtf(variance_accel[2]);
      const float std_gyro_x  = sqrtf(variance_gyro[0]);
      const float std_gyro_y  = sqrtf(variance_gyro[1]);
      const float std_gyro_z  = sqrtf(variance_gyro[2]);

      m_calibration.std_accel = Vec3{std_accel_x, std_accel_y, std_accel_z};
      m_calibration.std_gyro  = Vec3{std_gyro_x, std_gyro_y, std_gyro_z};
   }

   void calculate_bias()
   {
      const float magnitude = m_calibration.mean_accel.norm();
      error::verify(magnitude > 0);

      // unit vector -> a^ = a/|a|
      // gravity_body vector -> G_b = G * a^
      const Vec3 gravity_body = m_calibration.mean_accel * (math::constants::g_to_mps2 / magnitude);

      // bias = a - G_b
      // accel bias is the residual vector after subtracting the G component
      m_bias.accel = m_calibration.mean_accel - gravity_body;

      // gyro bias is just mean
      m_bias.gyro = m_calibration.mean_gyro;

      m_logger.printf("bias accel x: %.2f, y: %.2f, z: %.2f | bias gyro x :  %.2f, y: %.2f, z: %.2f",
                      m_bias.accel[0],
                      m_bias.accel[1],
                      m_bias.accel[2],
                      m_bias.gyro[0],
                      m_bias.gyro[1],
                      m_bias.gyro[2]);
   }

   void set_calibration_failure_error()
   {
      if (!is_platform_stationary())
      {
         set_error(imu::ImuError::non_stationary_calibration_error);
      }

      if (!is_accel_stable())
      {
         set_error(imu::ImuError::unstable_accel_error);
      }

      if (!is_gyro_stable())
      {
         set_error(imu::ImuError::unstable_gyro_error);
      }
   }

   void reset_calibration_failure_errors()
   {
      m_imu_status.error.reset();
   }

   void set_error(const imu::ImuError& error)
   {
      m_imu_status.error.set(static_cast<types::ErrorBitsType>(error));

      switch (error)
      {
         case imu::ImuError::non_stationary_calibration_error:
            m_logger.print("encountered error->non_stationary_calibration_error");
            break;
         case imu::ImuError::unstable_accel_error:
            m_logger.print("encountered error->unstable_accel_error");
            break;
         case imu::ImuError::unstable_gyro_error:
            m_logger.print("encountered error->unstable_gyro_error");
            break;
         case imu::ImuError::max_error:
         default:
            m_logger.print("encountered unexpected error");
            error::stop_operation();
            break;
      }
   }

   bool is_stale(const imu_sensor::RawImuSensorData& raw_data) const
   {
      return (raw_data.count <= m_previous_raw_sensor_data.count);
   }

   bool all_samples_collected() const
   {
      return (m_calibration.sample_counter >= m_num_calibration_samples);
   }

   bool is_calibration_valid() const
   {
      return (is_platform_stationary() && is_accel_stable() && is_gyro_stable());
   }

   bool is_platform_stationary() const
   {
      const float magnitude = m_calibration.mean_accel.norm();
      return std::abs(magnitude - math::constants::g_to_mps2) <= m_accel_tolerance_mps2;
   }

   bool is_accel_stable() const
   {
      return ((m_calibration.std_accel[0] <= m_accel_tolerance_mps2) &&
              (m_calibration.std_accel[1] <= m_accel_tolerance_mps2) &&
              (m_calibration.std_accel[2] <= m_accel_tolerance_mps2));
   }

   bool is_gyro_stable() const
   {
      return ((m_calibration.std_gyro[0] <= m_gyro_tolerance_radps) &&
              (m_calibration.std_gyro[1] <= m_gyro_tolerance_radps) &&
              (m_calibration.std_gyro[2] <= m_gyro_tolerance_radps));
   }

   struct Calibration
   {
      enum class State : uint8_t
      {
         stopped = 0,
         started,
         running,
         evaluating
      };

      Vec3 sum_accel{};
      Vec3 sum_sq_accel{};
      Vec3 mean_accel{};
      Vec3 std_accel{};

      Vec3 sum_gyro{};
      Vec3 sum_sq_gyro{};
      Vec3 mean_gyro{};
      Vec3 std_gyro{};

      State    state{State::stopped};
      uint32_t sample_counter{};

      void reset()
      {
         sum_accel.zero();
         sum_sq_accel.zero();
         mean_accel.zero();
         std_accel.zero();

         sum_gyro.zero();
         sum_sq_gyro.zero();
         mean_gyro.zero();
         std_gyro.zero();

         sample_counter = 0;
      }
   };

   struct Bias
   {
      Vec3 accel{};
      Vec3 gyro{};
   };

   ImuDataPublisher&            m_imu_data_publisher;
   ImuStatusPublisher&          m_imu_health_publisher;
   Logger&                      m_logger;
   const uint16_t               m_num_calibration_samples;
   const float                  m_gyro_tolerance_radps;
   const float                  m_accel_tolerance_mps2;
   const bool                   m_front_left_up_frame;
   imu::ImuData                 m_imu_data{};
   imu::ImuStatus               m_imu_status{};
   Calibration                  m_calibration{};
   Bias                         m_bias{};
   imu_sensor::RawImuSensorData m_previous_raw_sensor_data{};
   uint32_t                     m_clock_ms{};
};

}   // namespace imu

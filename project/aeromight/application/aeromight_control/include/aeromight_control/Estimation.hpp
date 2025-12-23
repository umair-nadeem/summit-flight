#pragma once

#include "StateEstimation.hpp"
#include "aeromight_boundaries/EstimatorHealth.hpp"
#include "barometer_sensor/BarometerData.hpp"
#include "bmp390/params.hpp"
#include "boundaries/SharedData.hpp"
#include "error/error_handler.hpp"
#include "imu_sensor/ImuData.hpp"
#include "interfaces/IClockSource.hpp"
#include "math/utility.hpp"
#include "utilities/Barometric.hpp"

namespace aeromight_control
{

template <typename AttitudeEstimator, typename AltitudeEkf, interfaces::IClockSource ClockSource, typename Logger>
class Estimation
{
   using ImuData         = ::boundaries::SharedData<imu_sensor::ImuData>;
   using BarometerData   = ::boundaries::SharedData<barometer_sensor::BarometerData>;
   using EstimatorHealth = ::boundaries::SharedData<aeromight_boundaries::EstimatorHealth>;
   using State           = aeromight_boundaries::EstimatorState;
   using Error           = aeromight_boundaries::EstimatorHealth::Error;

public:
   explicit Estimation(AttitudeEstimator&   attitude_estimator,
                       AltitudeEkf&         altitude_ekf,
                       EstimatorHealth&     estimator_health_storage,
                       StateEstimation&     state_estimation_data_storage,
                       const ImuData&       imu_data,
                       const BarometerData& barometer_data,
                       Logger&              logger,
                       const uint8_t        max_recovery_attempts,
                       const uint8_t        num_samples_reference_pressure,
                       const uint32_t       wait_timeout_reference_pressure_acquisition_ms,
                       const uint32_t       max_age_imu_data_ms,
                       const uint32_t       max_age_baro_data_ms,
                       const float          max_valid_imu_sample_dt_s,
                       const float          max_valid_barometer_sample_dt_s)
       : m_attitude_estimator{attitude_estimator},
         m_altitude_ekf{altitude_ekf},
         m_estimator_health_storage{estimator_health_storage},
         m_state_estimation_data_storage{state_estimation_data_storage},
         m_imu_data{imu_data},
         m_barometer_data{barometer_data},
         m_logger{logger},
         m_max_recovery_attempts{max_recovery_attempts},
         m_num_samples_reference_pressure{num_samples_reference_pressure},
         m_wait_timeout_reference_pressure_acquisition_ms{wait_timeout_reference_pressure_acquisition_ms},
         m_max_age_imu_data_ms{max_age_imu_data_ms},
         m_max_age_baro_data_ms{max_age_baro_data_ms},
         m_max_valid_imu_sample_dt_s{max_valid_imu_sample_dt_s},
         m_max_valid_barometer_sample_dt_s{max_valid_barometer_sample_dt_s}
   {
      m_logger.enable();
   }

   void start()
   {
      if (m_local_estimator_health.state == State::idle)
      {
         get_time();
         reset();
         m_local_estimator_health.state = State::get_reference_pressure;
         publish_health();
         m_logger.print("started");
      }
   }

   State get_state() const
   {
      return m_local_estimator_health.state;
   }

   float get_reference_pressure() const
   {
      return m_reference_pressure;
   }

   void execute()
   {
      get_time();

      run_state_machine();
   }

private:
   void get_time()
   {
      m_current_time_ms = ClockSource::now_ms();
   }

   void run_state_machine()
   {
      switch (m_local_estimator_health.state)
      {
         case State::idle:
            break;

         case State::get_reference_pressure:

            try_read_pressure();

            if (all_pressure_samples_acquired())
            {
               estimate_reference_pressure();

               if (reference_pressure_is_valid())
               {
                  utilities::Barometric::set_reference_pressure(m_reference_pressure);
                  m_local_estimator_health.valid_reference_pressure_acquired = true;
                  m_local_estimator_health.state                             = State::running;
               }
               else
               {
                  m_local_estimator_health.error.set(static_cast<uint8_t>(Error::reference_pressure_implausible));
                  move_to_fault();
               }
               publish_health();
            }
            else if (pressure_samples_read_timeout())
            {
               m_local_estimator_health.error.set(static_cast<uint8_t>(Error::reference_pressure_estimate_timeout));
               move_to_fault();
               publish_health();
            }
            break;

         case State::running:
            run_estimation();

            if (imu_data_is_stale())
            {
               m_local_estimator_health.error.set(static_cast<uint8_t>(Error::stale_imu_sensor_data));
               move_to_fault();
            }

            if (baro_data_is_stale())
            {
               m_local_estimator_health.error.set(static_cast<uint8_t>(Error::stale_baro_sensor_data));
            }

            publish_health();
            break;

         case State::fault:
            attempt_recovery_if_possible();
            break;

         default:
            error::stop_operation();
            break;
      }
   }

   void reset()
   {
      m_accumulated_pressure_values                 = 0.0f;
      m_reference_pressure                          = 0.0f;
      m_pressure_sample_counter                     = 0;
      m_reference_pressure_estimation_start_time_ms = m_current_time_ms;
      m_imu_sample_dt_s                             = 0.0f;
      m_last_barometer_sample                       = BarometerData::Sample{};
      m_last_imu_sample                             = ImuData::Sample{};

      m_attitude_estimator.reset();
      m_altitude_ekf.reset();
   }

   void try_read_pressure()
   {
      if (read_from_barometer())
      {
         m_accumulated_pressure_values += m_last_barometer_sample.data.pressure_pa.value();
         m_pressure_sample_counter++;
      }
   }

   void estimate_reference_pressure()
   {
      error::verify(m_pressure_sample_counter > 0);

      // take average and use as reference ground level pressure
      m_reference_pressure = m_accumulated_pressure_values / static_cast<float>(m_pressure_sample_counter);
      m_logger.printf("reference sea-level pressure: %.2f", m_reference_pressure);
   }

   void move_to_fault()
   {
      m_local_estimator_health.state = State::fault;
      m_logger.print("moving to fault");
   }

   void attempt_recovery_if_possible()
   {
      if (m_local_estimator_health.recovery_attempts < m_max_recovery_attempts)
      {
         // check for valid pressure reference
         if (m_local_estimator_health.valid_reference_pressure_acquired)
         {
            m_local_estimator_health.state = State::running;
         }
         else
         {
            reset();
            m_local_estimator_health.state = State::get_reference_pressure;
         }

         m_local_estimator_health.recovery_attempts++;
         publish_health();
         m_logger.printf("fault recovery attempt: %u", m_local_estimator_health.recovery_attempts);
      }
   }

   void run_estimation()
   {
      bool data_to_be_published = false;

      // read imu data
      if (read_from_imu())
      {
         run_attitude_estimation();

         data_to_be_published = true;
      }

      // read barometer data
      if (read_from_barometer())
      {
         run_altitude_estimation();

         data_to_be_published = true;
      }

      if (data_to_be_published)
      {
         publish_data();
      }
   }

   void run_attitude_estimation()
   {
      if (m_imu_sample_dt_s > m_max_valid_imu_sample_dt_s)
      {
         m_attitude_estimator.reset();
         m_altitude_ekf.reset();
         return;
      }

      // imu sensor mpu6500 gives data in FLU body frame
      const math::Vector3 accel_flu = m_last_imu_sample.data.accel_mps2.value();
      const math::Vector3 gyro_flu  = m_last_imu_sample.data.gyro_radps.value();

      // convert sensor axes from FLU body frame to FRD
      const math::Vector3 accel_mps2{map_imu_sensor_axes_to_frd(accel_flu)};
      const math::Vector3 gyro_radps{map_imu_sensor_axes_to_frd(gyro_flu)};

      // update attitude state estimation
      m_attitude_estimator.update(accel_mps2, gyro_radps, m_imu_sample_dt_s);

      m_local_estimation_data.attitude   = m_attitude_estimator.get_quaternion();
      m_local_estimation_data.gyro_radps = m_attitude_estimator.get_unbiased_gyro_data(gyro_radps);
      m_local_estimation_data.gyro_bias  = m_attitude_estimator.get_gyro_bias();
      m_local_estimation_data.euler      = math::quaternion_to_euler(m_local_estimation_data.attitude);

      // predict altitude with ekf2
      m_altitude_ekf.predict(accel_mps2, m_local_estimation_data.attitude, m_imu_sample_dt_s);
   }

   void run_altitude_estimation()
   {
      if (m_barometer_sample_dt_s > m_max_valid_barometer_sample_dt_s)
      {
         m_altitude_ekf.reset();
         return;
      }

      const float pressure_pa = m_last_barometer_sample.data.pressure_pa.value();
      const auto  altitude_m  = utilities::Barometric::convert_pressure_to_altitude(pressure_pa);

      // update altitude estimate of ekf2
      m_altitude_ekf.update(altitude_m);

      const auto altitude_state = m_altitude_ekf.get_ekf_state();

      m_local_estimation_data.altitude          = altitude_state.z;
      m_local_estimation_data.vertical_velocity = altitude_state.v_z;
   }

   void publish_data()
   {
      m_local_estimation_data.timestamp_ms = m_current_time_ms;
      m_state_estimation_data_storage      = m_local_estimation_data;

      // m_data_log_counter++;
      // if ((m_data_log_counter % 50u) == 0)
      // {
      //    m_logger.printf("z %.2f | vz %.2f | w %.2f | x %.2f | y %.2f | z %.2f | roll %.2f  |  pitch %.2f  |  y %.2f",
      //                    m_local_estimation_data.altitude,
      //                    m_local_estimation_data.vertical_velocity,
      //                    m_local_estimation_data.attitude.w,
      //                    m_local_estimation_data.attitude.x,
      //                    m_local_estimation_data.attitude.y,
      //                    m_local_estimation_data.attitude.z,
      //                    m_local_estimation_data.euler.x,
      //                    m_local_estimation_data.euler.y,
      //                    m_local_estimation_data.euler.z);
      // }
   }

   void publish_health()
   {
      m_estimator_health_storage.update_latest(m_local_estimator_health, m_current_time_ms);
   }

   bool read_from_imu()
   {
      const auto imu_sample = m_imu_data.get_latest();
      if (imu_sample.timestamp_ms > m_last_imu_sample.timestamp_ms)
      {
         // if this is our first sample
         if (m_last_imu_sample.timestamp_ms == 0)
         {
            m_last_imu_sample = imu_sample;
            return false;
         }

         // check for valid data
         if (imu_sample.data.accel_mps2.has_value() && imu_sample.data.gyro_radps.has_value())
         {
            m_imu_sample_dt_s = static_cast<float>(imu_sample.timestamp_ms - m_last_imu_sample.timestamp_ms) * 0.001f;
            m_last_imu_sample = imu_sample;
            return true;
         }
         else
         {
            m_local_estimator_health.error.set(static_cast<uint8_t>(Error::missing_valid_imu_data));
         }
      }

      return false;
   }

   bool read_from_barometer()
   {
      const auto barometer_sample = m_barometer_data.get_latest();
      if (barometer_sample.timestamp_ms > m_last_barometer_sample.timestamp_ms)
      {
         if (barometer_sample.data.pressure_pa.has_value())
         {
            m_barometer_sample_dt_s = static_cast<float>(barometer_sample.timestamp_ms - m_last_barometer_sample.timestamp_ms) * 0.001f;
            m_last_barometer_sample = barometer_sample;
            return true;
         }
         else
         {
            m_local_estimator_health.error.set(static_cast<uint8_t>(Error::missing_valid_baro_data));
         }
      }

      return false;
   }

   bool all_pressure_samples_acquired() const
   {
      return (m_pressure_sample_counter >= m_num_samples_reference_pressure);
   }

   bool pressure_samples_read_timeout() const
   {
      return ((m_current_time_ms - m_reference_pressure_estimation_start_time_ms) >= m_wait_timeout_reference_pressure_acquisition_ms);
   }

   bool reference_pressure_is_valid() const
   {
      return ((bmp390::params::min_plauisble_range_pressure_pa <= m_reference_pressure) &&
              (m_reference_pressure <= bmp390::params::max_plauisble_range_pressure_pa));
   }

   bool imu_data_is_stale() const
   {
      // Check IMU data freshness
      const uint32_t imu_data_age_ms = m_current_time_ms - m_last_imu_sample.timestamp_ms;
      if (imu_data_age_ms > m_max_age_imu_data_ms)
      {
         m_logger.printf("imu data stale (age: %u ms)", imu_data_age_ms);
         return true;
      }

      return false;
   }

   bool baro_data_is_stale() const
   {
      // Check barometer data freshness
      const uint32_t baro_data_age_ms = m_current_time_ms - m_last_barometer_sample.timestamp_ms;
      if (baro_data_age_ms > m_max_age_baro_data_ms)
      {
         return true;
      }

      return false;
   }

   // converts sensor axes from Front-Left-Up to Front_Right-Down
   static constexpr auto map_imu_sensor_axes_to_frd(const math::Vector3& vector3_flu)
   {
      return math::Vector3{-vector3_flu[1], -vector3_flu[0], -vector3_flu[2]};
   }

   AttitudeEstimator&                    m_attitude_estimator;
   AltitudeEkf&                          m_altitude_ekf;
   EstimatorHealth&                      m_estimator_health_storage;
   StateEstimation&                      m_state_estimation_data_storage;
   const ImuData&                        m_imu_data;
   const BarometerData&                  m_barometer_data;
   Logger&                               m_logger;
   const uint8_t                         m_max_recovery_attempts;
   const uint8_t                         m_num_samples_reference_pressure;
   const uint32_t                        m_wait_timeout_reference_pressure_acquisition_ms;
   const uint32_t                        m_max_age_imu_data_ms;
   const uint32_t                        m_max_age_baro_data_ms;
   const float                           m_max_valid_imu_sample_dt_s;
   const float                           m_max_valid_barometer_sample_dt_s;
   aeromight_boundaries::EstimatorHealth m_local_estimator_health{};
   StateEstimation                       m_local_estimation_data{};
   ImuData::Sample                       m_last_imu_sample{};
   BarometerData::Sample                 m_last_barometer_sample{};
   float                                 m_imu_sample_dt_s{};
   float                                 m_barometer_sample_dt_s{};
   float                                 m_accumulated_pressure_values{};
   float                                 m_reference_pressure{};
   uint8_t                               m_pressure_sample_counter{0};
   uint32_t                              m_current_time_ms{0};
   uint32_t                              m_reference_pressure_estimation_start_time_ms{0};
   std::size_t                           m_data_log_counter{0};
};

}   // namespace aeromight_control

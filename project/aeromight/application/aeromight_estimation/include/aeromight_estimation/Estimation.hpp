#pragma once

#include "aeromight_boundaries/EstimatorStatus.hpp"
#include "aeromight_boundaries/StateEstimation.hpp"
#include "barometer/BarometerData.hpp"
#include "boundaries/SharedData.hpp"
#include "error/error_handler.hpp"
#include "imu/ImuData.hpp"
#include "interfaces/IClockSource.hpp"
#include "math/utility.hpp"

namespace aeromight_estimation
{

template <typename AttitudeEstimator, typename AltitudeEkf, interfaces::IClockSource ClockSource, typename Logger>
class Estimation
{
   using EstimatorHealthPublisher = boundaries::SharedData<aeromight_boundaries::EstimatorStatus>;
   using ImuDataSubscriber        = boundaries::SharedData<imu::ImuData>;
   using BarometerDataSubscriber  = boundaries::SharedData<barometer::BarometerData>;
   using Error                    = aeromight_boundaries::EstimatorStatus::Error;

public:
   explicit Estimation(AttitudeEstimator&                     attitude_estimator,
                       AltitudeEkf&                           altitude_ekf,
                       EstimatorHealthPublisher&              estimator_health_publisher,
                       aeromight_boundaries::StateEstimation& state_estimation_data_publisher,
                       const ImuDataSubscriber&               imu_data_subscriber,
                       const BarometerDataSubscriber&         barometer_data_subscriber,
                       Logger&                                logger,
                       const bool                             run_altitude_estimation,
                       const uint32_t                         max_age_imu_data_ms,
                       const uint32_t                         max_age_baro_data_ms,
                       const float                            max_valid_imu_sample_dt_s,
                       const float                            max_valid_barometer_sample_dt_s)
       : m_attitude_estimator{attitude_estimator},
         m_altitude_ekf{altitude_ekf},
         m_estimator_health_publisher{estimator_health_publisher},
         m_state_estimation_data_publisher{state_estimation_data_publisher},
         m_imu_data_subscriber{imu_data_subscriber},
         m_barometer_data_subscriber{barometer_data_subscriber},
         m_logger{logger},
         m_run_altitude_estimation{run_altitude_estimation},
         m_max_age_imu_data_ms{max_age_imu_data_ms},
         m_max_age_baro_data_ms{max_age_baro_data_ms},
         m_max_valid_imu_sample_dt_s{max_valid_imu_sample_dt_s},
         m_max_valid_barometer_sample_dt_s{max_valid_barometer_sample_dt_s}
   {
      m_logger.enable();
   }

   void start()
   {
      m_estimation_status.enabled = true;
      get_time();
      reset();
      publish_health();
      m_logger.print("started");
   }

   void execute()
   {
      if (!m_estimation_status.enabled)
      {
         return;
      }

      get_time();

      m_estimation_status.error.reset();

      run_attitude_estimation();

      if (m_run_altitude_estimation)
      {
         run_altitude_estimation();
      }

      publish_data();

      publish_health();
   }

private:
   void get_time()
   {
      m_current_time_ms = ClockSource::now_ms();
   }

   void run_attitude_estimation()
   {
      const auto imu_sample = m_imu_data_subscriber.get_latest();

      if (m_previous_imu_sample.timestamp_ms == 0)   // first sample
      {
         m_previous_imu_sample = imu_sample;
         return;
      }

      const bool imu_valid = ((imu_sample.timestamp_ms > m_previous_imu_sample.timestamp_ms) &&
                              (imu_sample.data.accel_mps2.has_value() && imu_sample.data.gyro_radps.has_value()));

      const bool imu_current = imu_data_is_current(imu_sample);

      if (imu_valid && imu_current)
      {
         update_attitude_state(imu_sample, static_cast<float>(imu_sample.timestamp_ms - m_previous_imu_sample.timestamp_ms) * 0.001f);
         m_state_estimation.attitude_estimation_valid = true;
      }
      else
      {
         if (!imu_valid)
         {
            m_estimation_status.error.set(static_cast<types::ErrorBitsType>(Error::missing_valid_imu_data));
         }

         if (!imu_current)
         {
            m_estimation_status.error.set(static_cast<types::ErrorBitsType>(Error::stale_imu_sensor_data));
         }

         m_state_estimation.attitude_estimation_valid = false;
      }

      m_previous_imu_sample = imu_sample;
   }

   void run_altitude_estimation()
   {
      const auto barometer_sample = m_barometer_data_subscriber.get_latest();

      const bool baro_valid = ((barometer_sample.timestamp_ms > m_previous_barometer_sample.timestamp_ms) &&
                               barometer_sample.data.altitude_m.has_value());

      const bool baro_current = baro_data_is_current(barometer_sample);

      if (baro_valid && baro_current)
      {
         update_altitude_state(barometer_sample, static_cast<float>(barometer_sample.timestamp_ms - m_previous_barometer_sample.timestamp_ms) * 0.001f);
         m_state_estimation.altitude_estimation_valid = true;
      }
      else
      {
         if (!baro_valid)
         {
            m_estimation_status.error.set(static_cast<types::ErrorBitsType>(Error::missing_valid_baro_data));
         }

         if (!baro_current)
         {
            m_estimation_status.error.set(static_cast<types::ErrorBitsType>(Error::stale_baro_sensor_data));
         }

         m_state_estimation.altitude_estimation_valid = false;
      }

      m_previous_barometer_sample = barometer_sample;
   }

   void update_attitude_state(const ImuDataSubscriber::Sample& imu_sample, const float dt_s)
   {
      if (dt_s > m_max_valid_imu_sample_dt_s)
      {
         m_attitude_estimator.reset();
         m_altitude_ekf.reset();
         return;
      }

      // update attitude state estimation
      const math::Vector3 accel_mps2 = imu_sample.data.accel_mps2.value();
      const math::Vector3 gyro_radps = imu_sample.data.gyro_radps.value();
      m_attitude_estimator.update(accel_mps2, gyro_radps, dt_s);

      const auto attitude_q             = m_attitude_estimator.get_quaternion();
      m_state_estimation.euler          = math::quaternion_to_euler(attitude_q);
      m_state_estimation.raw_accel_mps2 = accel_mps2;
      m_state_estimation.raw_gyro_radps = gyro_radps;
      m_state_estimation.gyro_bias      = m_attitude_estimator.get_gyro_bias();

      if (m_run_altitude_estimation)
      {
         // predict altitude with ekf2
         m_altitude_ekf.predict(accel_mps2, attitude_q, dt_s);
      }
   }

   void update_altitude_state(const BarometerDataSubscriber::Sample& barometer_sample, const float dt_s)
   {
      if (dt_s > m_max_valid_barometer_sample_dt_s)
      {
         m_altitude_ekf.reset();
         return;
      }

      // update altitude estimate of ekf2
      const float altitude_m = barometer_sample.data.altitude_m.value();
      m_altitude_ekf.update(altitude_m);

      const auto altitude_state            = m_altitude_ekf.get_ekf_state();
      m_state_estimation.altitude          = altitude_state.z;
      m_state_estimation.vertical_velocity = altitude_state.v_z;
   }

   void reset()
   {
      m_attitude_estimator.reset();
      m_altitude_ekf.reset();
      m_previous_imu_sample       = ImuDataSubscriber::Sample{};
      m_previous_barometer_sample = BarometerDataSubscriber::Sample{};
   }

   void publish_data()
   {
      m_state_estimation.timestamp_ms   = m_current_time_ms;
      m_state_estimation_data_publisher = m_state_estimation;
   }

   void publish_health()
   {
      m_estimator_health_publisher.update_latest(m_estimation_status, m_current_time_ms);
   }

   bool imu_data_is_current(const ImuDataSubscriber::Sample& imu_sample) const
   {
      const uint32_t imu_data_age_ms = m_current_time_ms - imu_sample.timestamp_ms;
      return (imu_data_age_ms <= m_max_age_imu_data_ms);
   }

   bool baro_data_is_current(const BarometerDataSubscriber::Sample& barometer_sample) const
   {
      const uint32_t baro_data_age_ms = m_current_time_ms - barometer_sample.timestamp_ms;
      return (baro_data_age_ms <= m_max_age_baro_data_ms);
   }

   AttitudeEstimator&                     m_attitude_estimator;
   AltitudeEkf&                           m_altitude_ekf;
   EstimatorHealthPublisher&              m_estimator_health_publisher;
   aeromight_boundaries::StateEstimation& m_state_estimation_data_publisher;
   const ImuDataSubscriber&               m_imu_data_subscriber;
   const BarometerDataSubscriber&         m_barometer_data_subscriber;
   Logger&                                m_logger;
   const bool                             m_run_altitude_estimation;
   const uint32_t                         m_max_age_imu_data_ms;
   const uint32_t                         m_max_age_baro_data_ms;
   const float                            m_max_valid_imu_sample_dt_s;
   const float                            m_max_valid_barometer_sample_dt_s;
   aeromight_boundaries::EstimatorStatus  m_estimation_status{};
   aeromight_boundaries::StateEstimation  m_state_estimation{};
   ImuDataSubscriber::Sample              m_previous_imu_sample{};
   BarometerDataSubscriber::Sample        m_previous_barometer_sample{};
   uint32_t                               m_current_time_ms{0};
};

}   // namespace aeromight_estimation

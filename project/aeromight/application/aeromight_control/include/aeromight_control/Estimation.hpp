#pragma once

#include "StateEstimation.hpp"
#include "aeromight_boundaries/EstimatorHealth.hpp"
#include "barometer_sensor/BarometerData.hpp"
#include "boundaries/SharedData.hpp"
#include "error/error_handler.hpp"
#include "imu_sensor/ImuData.hpp"
#include "interfaces/IClockSource.hpp"
#include "utilities/Barometric.hpp"

namespace aeromight_control
{

template <typename MadgwickFilter, interfaces::IClockSource ClockSource, typename Logger>
class Estimation
{
   using ImuData         = ::boundaries::SharedData<imu_sensor::ImuData>;
   using BarometerData   = ::boundaries::SharedData<barometer_sensor::BarometerData>;
   using EstimatorHealth = ::boundaries::SharedData<aeromight_boundaries::EstimatorHealth>;
   using State           = aeromight_boundaries::EstimatorState;
   using Status          = aeromight_boundaries::EstimatorHealth::Status;

public:
   explicit Estimation(MadgwickFilter&      ahrs_filter,
                       EstimatorHealth&     estimator_health_storage,
                       StateEstimation&     state_estimation_data_storage,
                       const ImuData&       imu_data,
                       const BarometerData& barometer_data,
                       Logger&              logger,
                       const uint8_t        max_recovery_attempts,
                       const uint8_t        num_samples_pressure_reference,
                       const std::size_t    execution_period_ms,
                       const std::size_t    wait_timeout_pressure_reference_acquisition_ms,
                       const std::size_t    max_age_imu_data_ms,
                       const std::size_t    max_age_baro_data_ms)
       : m_ahrs_filter{ahrs_filter},
         m_estimator_health_storage{estimator_health_storage},
         m_state_estimation_data_storage{state_estimation_data_storage},
         m_imu_data{imu_data},
         m_barometer_data{barometer_data},
         m_logger{logger},
         m_max_recovery_attempts{max_recovery_attempts},
         m_num_samples_pressure_reference{num_samples_pressure_reference},
         m_execution_period_ms{execution_period_ms},
         m_wait_timeout_pressure_reference_acquisition_ms{wait_timeout_pressure_reference_acquisition_ms},
         m_max_age_imu_data_ms{max_age_imu_data_ms},
         m_max_age_baro_data_ms{max_age_baro_data_ms}
   {
      m_logger.enable();
   }

   void start()
   {
      m_local_estimator_health.state = State::get_pressure_reference;
      reset();
      publish_health();
      m_logger.print("started");
   }

   void stop()
   {
      m_local_estimator_health.state = State::idle;
      publish_health();
      m_logger.print("stopped");
   }

   State get_state() const
   {
      return m_local_estimator_health.state;
   }

   void execute()
   {
      switch (m_local_estimator_health.state)
      {
         case State::idle:
            break;

         case State::get_pressure_reference:

            m_pressure_reference_wait_timer += m_execution_period_ms;
            try_read_pressure();

            if (all_pressure_samples_acquired())
            {
               estimate_pressure_reference();
               m_local_estimator_health.state = State::running;
               publish_health();
            }
            else if (pressure_samples_read_timeout())
            {
               move_to_fault();
            }
            break;

         case State::running:
            run_estimation();

            if (estimation_has_fault())
            {
               move_to_fault();
            }
            break;

         case State::fault:
            attempt_recovery_if_possible();
            break;

         default:
            error::stop_operation();
            break;
      }
   }

private:
   void reset()
   {
      m_accumulated_pressure_values   = 0.0f;
      m_reference_pressure            = 0.0f;
      m_pressure_sample_counter       = 0;
      m_pressure_reference_wait_timer = 0;
      m_last_barometer_sample         = BarometerData::Sample{};
      m_last_imu_sample               = ImuData::Sample{};
   }

   void try_read_pressure()
   {
      const auto pressure_sample = m_barometer_data.get_latest();
      if (pressure_sample.timestamp_ms > m_last_barometer_sample.timestamp_ms)
      {
         if (pressure_sample.data.pressure_pa.has_value())
         {
            m_accumulated_pressure_values += pressure_sample.data.pressure_pa.value();
            m_last_barometer_sample = pressure_sample;
            m_pressure_sample_counter++;
         }
      }
   }

   void estimate_pressure_reference()
   {
      error::verify(m_pressure_sample_counter > 0);

      // take average and use as reference ground level pressure
      m_reference_pressure = m_accumulated_pressure_values / static_cast<float>(m_pressure_sample_counter);
      utilities::Barometric::set_pressure_reference(m_reference_pressure);
      m_local_estimator_health.status.set(static_cast<uint8_t>(Status::valid_pressure_reference_acquired));
      m_logger.printf("reference sea-level pressure: %.2f", m_reference_pressure);
   }

   void publish_health()
   {
      m_estimator_health_storage.update_latest(m_local_estimator_health, ClockSource::now_ms());
   }

   void move_to_fault()
   {
      m_local_estimator_health.state = State::fault;
      m_local_estimator_health.status.set(static_cast<uint8_t>(Status::fault_occurred));
      publish_health();
   }

   void attempt_recovery_if_possible()
   {
      if (m_local_estimator_health.recovery_attempts < m_max_recovery_attempts)
      {
         // check for valid pressure reference
         if (m_local_estimator_health.status.test(static_cast<uint8_t>(Status::valid_pressure_reference_acquired)))
         {
            m_local_estimator_health.state = State::running;
         }
         else
         {
            m_local_estimator_health.state = State::get_pressure_reference;
            reset();
         }

         m_local_estimator_health.recovery_attempts++;
         publish_health();
         m_logger.printf("fault recovery attempt: %u", m_local_estimator_health.recovery_attempts);
      }
   }

   static void run_estimation()
   {
   }

   bool all_pressure_samples_acquired() const
   {
      return (m_pressure_sample_counter >= m_num_samples_pressure_reference);
   }

   bool pressure_samples_read_timeout() const
   {
      return (m_pressure_reference_wait_timer >= m_wait_timeout_pressure_reference_acquisition_ms);
   }

   bool estimation_has_fault() const
   {
      const uint32_t current_time = ClockSource::now_ms();

      // Check IMU data freshness
      const uint32_t imu_data_age_ms = current_time - m_last_imu_sample.timestamp_ms;
      if (imu_data_age_ms > m_max_age_imu_data_ms)
      {
         m_logger.printf("imu data stale (age: %u ms)", imu_data_age_ms);
         return true;
      }

      // Check barometer data freshness
      const uint32_t baro_data_age_ms = current_time - m_last_barometer_sample.timestamp_ms;
      if (baro_data_age_ms > m_max_age_baro_data_ms)
      {
         m_logger.printf("barometer data stale (age: %u ms)", baro_data_age_ms);
         return true;
      }

      return false;
   }

   MadgwickFilter&                       m_ahrs_filter;
   EstimatorHealth&                      m_estimator_health_storage;
   StateEstimation&                      m_state_estimation_data_storage;
   const ImuData&                        m_imu_data;
   const BarometerData&                  m_barometer_data;
   Logger&                               m_logger;
   const uint8_t                         m_max_recovery_attempts;
   const uint8_t                         m_num_samples_pressure_reference;
   const std::size_t                     m_execution_period_ms;
   const std::size_t                     m_wait_timeout_pressure_reference_acquisition_ms;
   const std::size_t                     m_max_age_imu_data_ms;
   const std::size_t                     m_max_age_baro_data_ms;
   aeromight_boundaries::EstimatorHealth m_local_estimator_health{};
   ImuData::Sample                       m_last_imu_sample{};
   BarometerData::Sample                 m_last_barometer_sample{};
   float                                 m_accumulated_pressure_values{};
   float                                 m_reference_pressure{};
   uint8_t                               m_pressure_sample_counter{};
   std::size_t                           m_pressure_reference_wait_timer{};
};

}   // namespace aeromight_control

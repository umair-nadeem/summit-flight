#pragma once

#include "HealthMonitoringState.hpp"
#include "aeromight_boundaries/EstimatorHealth.hpp"
#include "aeromight_boundaries/HealthSummary.hpp"
#include "barometer_sensor/BarometerHealth.hpp"
#include "boundaries/SharedData.hpp"
#include "imu_sensor/ImuHealth.hpp"
#include "interfaces/IClockSource.hpp"
#include "interfaces/rtos/IQueueSender.hpp"

namespace aeromight_health
{

template <interfaces::rtos::IQueueSender<aeromight_boundaries::HealthSummary> QueueSender,
          interfaces::IClockSource                                            ClockSource,
          typename Logger>
class HealthMonitoring
{
public:
   explicit HealthMonitoring(QueueSender                                                          queue_sender,
                             const boundaries::SharedData<imu_sensor::ImuHealth>&                 imu_sensor_health,
                             const boundaries::SharedData<barometer_sensor::BarometerHealth>&     barometer_sensor_health,
                             const boundaries::SharedData<aeromight_boundaries::EstimatorHealth>& estimation_health,
                             Logger&                                                              logger,
                             const std::size_t                                                    period_in_ms,
                             const std::size_t                                                    startup_wait_ms,
                             const std::size_t                                                    max_wait_sensors_readiness_ms,
                             const std::size_t                                                    max_wait_estimation_control_readiness_ms,
                             const std::size_t                                                    max_age_imu_sensor_health_ms,
                             const std::size_t                                                    max_age_barometer_sensor_health_ms,
                             const std::size_t                                                    max_age_estimation_health_ms,
                             const std::size_t                                                    max_age_control_health_ms)
       : m_queue_sender{queue_sender},
         m_imu_sensor_health{imu_sensor_health},
         m_barometer_sensor_health{barometer_sensor_health},
         m_estimation_health{estimation_health},
         m_logger{logger},
         m_period_in_ms{period_in_ms},
         m_startup_wait_ms{startup_wait_ms},
         m_max_wait_sensors_readiness_ms{max_wait_sensors_readiness_ms},
         m_max_wait_estimation_control_readiness_ms{max_wait_estimation_control_readiness_ms},
         m_max_age_imu_sensor_health_ms{max_age_imu_sensor_health_ms},
         m_max_age_barometer_sensor_health_ms{max_age_barometer_sensor_health_ms},
         m_max_age_estimation_health_ms{max_age_estimation_health_ms},
         m_max_age_control_health_ms{max_age_control_health_ms}
   {
      m_logger.enable();
   }

   void run_once()
   {
      const uint32_t current_time_ms = ClockSource::now_ms();

      switch (m_state)
      {
         case HealthMonitoringState::startup:

            tick_timer();
            if (startup_wait_duration_passed())
            {
               m_state = HealthMonitoringState::wait_for_sensors_readiness;
               reset_timer();
            }
            break;

         case HealthMonitoringState::wait_for_sensors_readiness:

            tick_timer();
            get_latest_health_snapshots();

            if (all_sensors_ready())
            {
               m_health_summary.imu_health        = aeromight_boundaries::SubsystemHealth::operational;
               m_health_summary.barometer_health  = aeromight_boundaries::SubsystemHealth::operational;
               m_health_summary.all_sensors_ready = true;
               m_state                            = HealthMonitoringState::wait_for_estimation_control_readiness;
               m_logger.print("sensors healthy");
               reset_timer();
            }
            else if (wait_for_sensors_readiness_passed())
            {
               m_health_summary.all_sensors_ready = false;
               if (m_imu_health_snapshot.data.state != imu_sensor::ImuSensorState::operational)
               {
                  m_health_summary.imu_health = aeromight_boundaries::SubsystemHealth::fault;
               }
               if (m_barometer_health_snapshot.data.state != barometer_sensor::BarometerSensorState::operational)
               {
                  m_health_summary.barometer_health = aeromight_boundaries::SubsystemHealth::fault;
               }
               m_state = HealthMonitoringState::wait_for_estimation_control_readiness;
               m_logger.print("sensors health couldn't be determined");
               reset_timer();
            }

            publish_health_summary(current_time_ms);
            break;

         case HealthMonitoringState::wait_for_estimation_control_readiness:
            tick_timer();
            get_latest_health_snapshots();

            if (estimation_control_is_ready())
            {
               m_health_summary.estimation_health = aeromight_boundaries::SubsystemHealth::operational;
               m_health_summary.estimation_ready  = true;
               m_health_summary.control_ready     = true;
               m_state                            = HealthMonitoringState::general_monitoring;
               m_logger.print("estimation & control healthy");
               reset_timer();
            }
            else if (wait_for_estimation_control_readiness_passed())
            {
               m_health_summary.estimation_health = aeromight_boundaries::SubsystemHealth::fault;
               m_health_summary.estimation_ready  = false;
               m_health_summary.control_ready     = false;
               m_state                            = HealthMonitoringState::general_monitoring;
               m_logger.print("estimation & control health couldn't be determined");
               reset_timer();
            }

            publish_health_summary(current_time_ms);
            break;

         case HealthMonitoringState::general_monitoring:

            get_latest_health_snapshots();
            evaluate_health_status(current_time_ms);
            publish_health_summary(current_time_ms);
            break;

         default:
            break;
      }
   }

   aeromight_boundaries::HealthSummary get_health_summary() const
   {
      return m_health_summary;
   }

   HealthMonitoringState get_state() const
   {
      return m_state;
   }

   std::size_t get_period_ms() const
   {
      return m_period_in_ms;
   }

private:
   void reset_timer()
   {
      m_wait_timer_ms = 0;
   }

   void tick_timer()
   {
      m_wait_timer_ms += m_period_in_ms;
   }

   void get_latest_health_snapshots()
   {
      m_imu_health_snapshot        = m_imu_sensor_health.get_latest();
      m_barometer_health_snapshot  = m_barometer_sensor_health.get_latest();
      m_estimation_health_snapshot = m_estimation_health.get_latest();
   }

   void evaluate_health_status(const uint32_t current_time_ms)
   {
      const bool critical_imu_fault        = evaluate_imu_health(current_time_ms);
      const bool critical_barometer_fault  = evaluate_barometer_health(current_time_ms);
      const bool critical_estimation_fault = evaluate_estimation_health(current_time_ms);

      if (critical_imu_fault || critical_barometer_fault || critical_estimation_fault)
      {
         m_health_summary.flight_critical_fault = true;
      }
      else
      {
         m_health_summary.flight_critical_fault = false;
      }
   }

   void publish_health_summary(const uint32_t current_time_ms)
   {
      m_health_summary.timestamp_ms                = current_time_ms;
      const bool ok                                = m_queue_sender.send_if_possible(m_health_summary);
      m_health_summary.health_update_queue_failure = !ok;
   }

   bool evaluate_imu_health(const uint32_t current_time_ms)
   {
      const bool imu_stale = (((current_time_ms - m_imu_health_snapshot.timestamp_ms) > m_max_age_imu_sensor_health_ms));

      const bool imu_read_failures = m_imu_health_snapshot.data.read_failure_count > 0;

      const bool imu_fault = ((m_imu_health_snapshot.data.state == imu_sensor::ImuSensorState::failure) ||
                              (m_imu_health_snapshot.data.error.to_ulong() != 0));

      if (imu_stale || imu_read_failures)
      {
         m_health_summary.imu_health = aeromight_boundaries::SubsystemHealth::degraded;
      }

      if (imu_fault)
      {
         m_health_summary.imu_health = aeromight_boundaries::SubsystemHealth::fault;   // overwrite with higher criticality
      }

      return imu_fault;
   }

   bool evaluate_barometer_health(const uint32_t current_time_ms)
   {
      const bool barometer_stale = (((current_time_ms - m_barometer_health_snapshot.timestamp_ms) > m_max_age_barometer_sensor_health_ms));

      const bool barometer_read_failures = m_barometer_health_snapshot.data.read_failure_count > 0;

      const bool barometer_fault = ((m_barometer_health_snapshot.data.state == barometer_sensor::BarometerSensorState::failure) ||
                                    (m_barometer_health_snapshot.data.error.to_ulong() != 0));

      if (barometer_stale || barometer_read_failures)
      {
         m_health_summary.barometer_health = aeromight_boundaries::SubsystemHealth::degraded;
      }

      if (barometer_fault)
      {
         m_health_summary.barometer_health = aeromight_boundaries::SubsystemHealth::fault;   // overwrite with higher criticality
      }

      return barometer_fault;
   }

   bool evaluate_estimation_health(const uint32_t current_time_ms)
   {
      using namespace aeromight_boundaries;
      using Status = EstimatorHealth::Status;

      const bool estimation_stale = (((current_time_ms - m_estimation_health_snapshot.timestamp_ms) > m_max_age_estimation_health_ms));

      const bool estimation_fault = ((m_estimation_health_snapshot.data.state == EstimatorState::fault) ||
                                     (m_estimation_health_snapshot.data.status.test(static_cast<uint8_t>(Status::stale_imu_sensor_data))) ||
                                     (m_estimation_health_snapshot.data.status.test(static_cast<uint8_t>(Status::stale_baro_sensor_data))) ||
                                     (m_estimation_health_snapshot.data.status.test(static_cast<uint8_t>(Status::missing_valid_imu_data))) ||
                                     (m_estimation_health_snapshot.data.status.test(static_cast<uint8_t>(Status::missing_valid_baro_data))));

      if (estimation_stale)
      {
         m_health_summary.estimation_health = SubsystemHealth::degraded;
      }

      if (estimation_fault)
      {
         m_health_summary.estimation_health = SubsystemHealth::fault;   // overwrite with higher criticality
      }

      return estimation_fault;
   }

   bool startup_wait_duration_passed() const
   {
      return m_wait_timer_ms >= m_startup_wait_ms;
   }

   bool wait_for_sensors_readiness_passed() const
   {
      return m_wait_timer_ms >= m_max_wait_sensors_readiness_ms;
   }

   bool wait_for_estimation_control_readiness_passed() const
   {
      return m_wait_timer_ms >= m_max_wait_estimation_control_readiness_ms;
   }

   bool all_sensors_ready() const
   {
      const auto& imu_health       = m_imu_health_snapshot.data;
      const auto& barometer_health = m_barometer_health_snapshot.data;

      const bool imu_ready = (imu_health.validation_ok &&
                              imu_health.config_ok &&
                              imu_health.self_test_ok &&
                              (imu_health.error.to_ulong() == 0) &&
                              (imu_health.state == imu_sensor::ImuSensorState::operational));

      const bool barometer_ready = (barometer_health.setup_ok &&
                                    (barometer_health.error.to_ulong() == 0) &&
                                    (barometer_health.state == barometer_sensor::BarometerSensorState::operational));

      return (imu_ready && barometer_ready);
   }

   bool estimation_control_is_ready() const
   {
      using namespace aeromight_boundaries;
      return (m_estimation_health_snapshot.data.status.test(static_cast<uint8_t>(EstimatorHealth::Status::valid_reference_pressure_acquired)) &&
              (m_estimation_health_snapshot.data.state == EstimatorState::running));
   }

   QueueSender                                                           m_queue_sender;
   const boundaries::SharedData<imu_sensor::ImuHealth>&                  m_imu_sensor_health;
   const boundaries::SharedData<barometer_sensor::BarometerHealth>&      m_barometer_sensor_health;
   const boundaries::SharedData<aeromight_boundaries::EstimatorHealth>&  m_estimation_health;
   Logger&                                                               m_logger;
   const std::size_t                                                     m_period_in_ms;
   const std::size_t                                                     m_startup_wait_ms;
   const std::size_t                                                     m_max_wait_sensors_readiness_ms;
   const std::size_t                                                     m_max_wait_estimation_control_readiness_ms;
   const std::size_t                                                     m_max_age_imu_sensor_health_ms;
   const std::size_t                                                     m_max_age_barometer_sensor_health_ms;
   const std::size_t                                                     m_max_age_estimation_health_ms;
   const std::size_t                                                     m_max_age_control_health_ms;
   aeromight_boundaries::HealthSummary                                   m_health_summary{};
   boundaries::SharedData<imu_sensor::ImuHealth>::Sample                 m_imu_health_snapshot{};
   boundaries::SharedData<barometer_sensor::BarometerHealth>::Sample     m_barometer_health_snapshot{};
   boundaries::SharedData<aeromight_boundaries::EstimatorHealth>::Sample m_estimation_health_snapshot{};
   HealthMonitoringState                                                 m_state{HealthMonitoringState::startup};
   std::size_t                                                           m_wait_timer_ms{};
};

}   // namespace aeromight_health

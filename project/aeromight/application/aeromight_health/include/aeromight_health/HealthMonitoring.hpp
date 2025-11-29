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
   explicit HealthMonitoring(QueueSender&                                                         queue_sender,
                             const boundaries::SharedData<imu_sensor::ImuHealth>&                 imu_sensor_health,
                             const boundaries::SharedData<barometer_sensor::BarometerHealth>&     barometer_sensor_health,
                             const boundaries::SharedData<aeromight_boundaries::EstimatorHealth>& estimation_health,
                             Logger&                                                              logger,
                             const uint32_t                                                       period_in_ms,
                             const uint32_t                                                       startup_wait_ms,
                             const uint32_t                                                       max_wait_sensors_readiness_ms,
                             const uint32_t                                                       max_wait_estimation_control_readiness_ms,
                             const uint32_t                                                       max_age_imu_sensor_health_ms,
                             const uint32_t                                                       max_age_barometer_sensor_health_ms,
                             const uint32_t                                                       max_age_estimation_health_ms,
                             const uint32_t                                                       max_age_control_health_ms)
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
      m_current_time_ms = ClockSource::now_ms();

      switch (m_state)
      {
         case HealthMonitoringState::startup:
         {
            if (startup_wait_duration_passed())
            {
               m_state = HealthMonitoringState::wait_for_sensors_readiness;
               m_logger.print("reading health snapshots");
               m_state_entry_time_ms = m_current_time_ms;
            }
            break;
         }

         case HealthMonitoringState::wait_for_sensors_readiness:
         {
            get_latest_health_snapshots();

            const bool imu_ready       = imu_is_ready();
            const bool barometer_ready = barometer_is_ready();

            if (imu_ready)
            {
               m_health_summary.imu_health = aeromight_boundaries::SubsystemHealth::operational;
            }

            if (barometer_ready)
            {
               m_health_summary.barometer_health = aeromight_boundaries::SubsystemHealth::operational;
            }

            if (imu_ready && barometer_ready)
            {
               m_health_summary.all_sensors_ready = true;
               m_state                            = HealthMonitoringState::wait_for_estimation_control_readiness;
               m_state_entry_time_ms              = m_current_time_ms;
               m_logger.print("sensors healthy");
            }
            else if (wait_for_sensors_readiness_passed())
            {
               if (!imu_ready)
               {
                  m_health_summary.imu_health = aeromight_boundaries::SubsystemHealth::fault;
               }

               if (!barometer_ready)
               {
                  m_health_summary.barometer_health = aeromight_boundaries::SubsystemHealth::fault;
               }
               m_health_summary.all_sensors_ready = false;
               m_state                            = HealthMonitoringState::wait_for_estimation_control_readiness;
               m_state_entry_time_ms              = m_current_time_ms;
               m_logger.print("sensors health couldn't be determined");
            }

            publish_health_summary();
            break;
         }

         case HealthMonitoringState::wait_for_estimation_control_readiness:
         {
            get_latest_health_snapshots();

            if (estimation_control_is_ready())
            {
               m_health_summary.estimation_health = aeromight_boundaries::SubsystemHealth::operational;
               m_health_summary.estimation_ready  = true;
               m_health_summary.control_ready     = true;
               m_state                            = HealthMonitoringState::general_monitoring;
               m_state_entry_time_ms              = m_current_time_ms;
               m_logger.print("estimation & control healthy");
            }
            else if (wait_for_estimation_control_readiness_passed())
            {
               m_health_summary.estimation_health = aeromight_boundaries::SubsystemHealth::fault;
               m_health_summary.estimation_ready  = false;
               m_health_summary.control_ready     = false;
               m_state                            = HealthMonitoringState::general_monitoring;
               m_state_entry_time_ms              = m_current_time_ms;
               m_logger.print("estimation & control health couldn't be determined");
            }

            publish_health_summary();
            break;
         }

         case HealthMonitoringState::general_monitoring:
         {
            get_latest_health_snapshots();
            evaluate_health_status();
            publish_health_summary();
            break;
         }

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

   uint32_t get_period_ms() const
   {
      return m_period_in_ms;
   }

private:
   void get_latest_health_snapshots()
   {
      m_imu_health_snapshot        = m_imu_sensor_health.get_latest();
      m_barometer_health_snapshot  = m_barometer_sensor_health.get_latest();
      m_estimation_health_snapshot = m_estimation_health.get_latest();
   }

   void evaluate_health_status()
   {
      using namespace aeromight_boundaries;

      m_health_summary.imu_health        = get_imu_health();
      m_health_summary.barometer_health  = get_barometer_health();
      m_health_summary.estimation_health = get_estimation_health();

      if ((m_health_summary.imu_health == SubsystemHealth::fault) ||
          (m_health_summary.estimation_health == SubsystemHealth::fault))
      {
         m_health_summary.flight_health = FlightHealthStatus::critical;
      }
      else if (m_health_summary.barometer_health == SubsystemHealth::fault)
      {
         m_health_summary.flight_health = FlightHealthStatus::degraded;
      }
      else
      {
         m_health_summary.flight_health = FlightHealthStatus::nominal;
      }
   }

   void publish_health_summary()
   {
      m_health_summary.timestamp_ms = m_current_time_ms;
      const bool ok                 = m_queue_sender.send_if_possible(m_health_summary);
      if (!ok)
      {
         m_health_summary.queue_failure_count++;
      }
   }

   aeromight_boundaries::SubsystemHealth get_imu_health() const
   {
      const bool imu_stale = (((m_current_time_ms - m_imu_health_snapshot.timestamp_ms) > m_max_age_imu_sensor_health_ms));

      const bool imu_read_failures = m_imu_health_snapshot.data.read_failure_count > 0;

      const bool imu_fault = ((m_imu_health_snapshot.data.state == imu_sensor::ImuSensorState::failure) ||
                              (m_imu_health_snapshot.data.error.to_ulong() != 0));

      if (imu_fault)
      {
         return aeromight_boundaries::SubsystemHealth::fault;
      }
      else if (imu_stale || imu_read_failures)
      {
         return aeromight_boundaries::SubsystemHealth::degraded;
      }

      return aeromight_boundaries::SubsystemHealth::operational;
   }

   aeromight_boundaries::SubsystemHealth get_barometer_health() const
   {
      const bool barometer_stale = (((m_current_time_ms - m_barometer_health_snapshot.timestamp_ms) > m_max_age_barometer_sensor_health_ms));

      const bool barometer_read_failures = m_barometer_health_snapshot.data.read_failure_count > 0;

      const bool barometer_fault = ((m_barometer_health_snapshot.data.state == barometer_sensor::BarometerSensorState::failure) ||
                                    (m_barometer_health_snapshot.data.error.to_ulong() != 0));

      if (barometer_fault)
      {
         return aeromight_boundaries::SubsystemHealth::fault;
      }
      else if (barometer_stale || barometer_read_failures)
      {
         return aeromight_boundaries::SubsystemHealth::degraded;
      }

      return aeromight_boundaries::SubsystemHealth::operational;
   }

   aeromight_boundaries::SubsystemHealth get_estimation_health() const
   {
      const bool estimation_stale = (((m_current_time_ms - m_estimation_health_snapshot.timestamp_ms) > m_max_age_estimation_health_ms));

      const bool estimation_fault = ((m_estimation_health_snapshot.data.state == aeromight_boundaries::EstimatorState::fault) ||
                                     (m_estimation_health_snapshot.data.error.to_ulong() != 0));
      if (estimation_fault)
      {
         return aeromight_boundaries::SubsystemHealth::fault;
      }
      else if (estimation_stale)
      {
         return aeromight_boundaries::SubsystemHealth::degraded;
      }

      return aeromight_boundaries::SubsystemHealth::operational;
   }

   bool startup_wait_duration_passed() const
   {
      return (m_current_time_ms - m_state_entry_time_ms) >= m_startup_wait_ms;
   }

   bool wait_for_sensors_readiness_passed() const
   {
      return (m_current_time_ms - m_state_entry_time_ms) >= m_max_wait_sensors_readiness_ms;
   }

   bool wait_for_estimation_control_readiness_passed() const
   {
      return (m_current_time_ms - m_state_entry_time_ms) >= m_max_wait_estimation_control_readiness_ms;
   }

   bool imu_is_ready() const
   {
      const auto& imu_health = m_imu_health_snapshot.data;

      return (imu_health.validation_ok &&
              imu_health.config_ok &&
              imu_health.self_test_ok &&
              (imu_health.error.to_ulong() == 0) &&
              (imu_health.state == imu_sensor::ImuSensorState::operational));
   }

   bool barometer_is_ready() const
   {
      const auto& barometer_health = m_barometer_health_snapshot.data;

      return (barometer_health.setup_ok &&
              (barometer_health.error.to_ulong() == 0) &&
              (barometer_health.state == barometer_sensor::BarometerSensorState::operational));
   }

   bool estimation_control_is_ready() const
   {
      return (m_estimation_health_snapshot.data.valid_reference_pressure_acquired &&
              (m_estimation_health_snapshot.data.state == aeromight_boundaries::EstimatorState::running));
   }

   QueueSender&                                                          m_queue_sender;
   const boundaries::SharedData<imu_sensor::ImuHealth>&                  m_imu_sensor_health;
   const boundaries::SharedData<barometer_sensor::BarometerHealth>&      m_barometer_sensor_health;
   const boundaries::SharedData<aeromight_boundaries::EstimatorHealth>&  m_estimation_health;
   Logger&                                                               m_logger;
   const uint32_t                                                        m_period_in_ms;
   const uint32_t                                                        m_startup_wait_ms;
   const uint32_t                                                        m_max_wait_sensors_readiness_ms;
   const uint32_t                                                        m_max_wait_estimation_control_readiness_ms;
   const uint32_t                                                        m_max_age_imu_sensor_health_ms;
   const uint32_t                                                        m_max_age_barometer_sensor_health_ms;
   const uint32_t                                                        m_max_age_estimation_health_ms;
   const uint32_t                                                        m_max_age_control_health_ms;
   aeromight_boundaries::HealthSummary                                   m_health_summary{};
   boundaries::SharedData<imu_sensor::ImuHealth>::Sample                 m_imu_health_snapshot{};
   boundaries::SharedData<barometer_sensor::BarometerHealth>::Sample     m_barometer_health_snapshot{};
   boundaries::SharedData<aeromight_boundaries::EstimatorHealth>::Sample m_estimation_health_snapshot{};
   HealthMonitoringState                                                 m_state{HealthMonitoringState::startup};
   uint32_t                                                              m_current_time_ms{0};
   uint32_t                                                              m_state_entry_time_ms{0};
};

}   // namespace aeromight_health

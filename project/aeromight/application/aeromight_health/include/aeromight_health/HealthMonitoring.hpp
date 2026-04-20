#pragma once

#include "aeromight_boundaries/ControlStatus.hpp"
#include "aeromight_boundaries/EstimatorStatus.hpp"
#include "aeromight_boundaries/HealthSummary.hpp"
#include "barometer/BarometerStatus.hpp"
#include "boundaries/SharedData.hpp"
#include "imu/ImuStatus.hpp"
#include "interfaces/IClockSource.hpp"
#include "interfaces/rtos/IQueueSender.hpp"
#include "power/battery/BatteryStatus.hpp"

namespace aeromight_health
{

template <typename Battery,
          interfaces::rtos::IQueueSender<aeromight_boundaries::HealthSummary> QueueSender,
          interfaces::IClockSource                                            ClockSource,
          typename Logger>
class HealthMonitoring
{
   using BatteryStatusPublisher = boundaries::SharedData<power::battery::BatteryStatus>;

public:
   explicit HealthMonitoring(Battery&                                                             battery,
                             QueueSender&                                                         queue_sender,
                             BatteryStatusPublisher&                                              battery_status_publisher,
                             const boundaries::SharedData<imu::ImuStatus>&                        imu_health_subscriber,
                             const boundaries::SharedData<barometer::BarometerStatus>&            barometer_health_subscriber,
                             const boundaries::SharedData<aeromight_boundaries::EstimatorStatus>& estimation_health_subscriber,
                             const boundaries::SharedData<aeromight_boundaries::ControlStatus>&   control_health_subscriber,
                             Logger&                                                              logger,
                             const uint32_t                                                       period_in_ms,
                             const uint32_t                                                       max_age_imu_health_ms,
                             const uint32_t                                                       max_age_barometer_health_ms,
                             const uint32_t                                                       max_age_estimation_health_ms,
                             const uint32_t                                                       max_age_control_health_ms,
                             const bool                                                           evaluate_barometer_health)
       : m_battery{battery},
         m_queue_sender{queue_sender},
         m_battery_status_publisher{battery_status_publisher},
         m_imu_health_subscriber{imu_health_subscriber},
         m_barometer_health_subscriber{barometer_health_subscriber},
         m_estimation_health_subscriber{estimation_health_subscriber},
         m_control_health_subscriber{control_health_subscriber},
         m_logger{logger},
         m_period_in_ms{period_in_ms},
         m_max_age_imu_health_ms{max_age_imu_health_ms},
         m_max_age_barometer_health_ms{max_age_barometer_health_ms},
         m_max_age_estimation_health_ms{max_age_estimation_health_ms},
         m_max_age_control_health_ms{max_age_control_health_ms},
         m_evaluate_barometer_health{evaluate_barometer_health}
   {
      m_logger.enable();
   }

   void run_once()
   {
      m_current_time_ms = ClockSource::now_ms();

      m_battery.execute();

      get_health_updates();

      publish_health_summary();

      publish_battery_status();
   }

   aeromight_boundaries::HealthSummary get_health_summary() const
   {
      return m_health_summary;
   }

   uint32_t get_period_ms() const
   {
      return m_period_in_ms;
   }

private:
   void get_health_updates()
   {
      evaluate_imu_health();
      evaluate_estimation_health();
      evaluate_control_health();

      if (m_evaluate_barometer_health)
      {
         evaluate_barometer_health();
      }
   }

   void evaluate_imu_health()
   {
      const auto imu_health = m_imu_health_subscriber.get_latest();

      m_health_summary.imu_operational = (imu_health.data.available && (imu_health.data.error.to_ulong() == 0));

      m_health_summary.imu_calibration_finished = imu_health.data.calibration_done;

      const bool imu_stale = (((m_current_time_ms - imu_health.timestamp_ms) > m_max_age_imu_health_ms));

      if (!m_health_summary.imu_operational)
      {
         m_health_summary.imu_health = aeromight_boundaries::SubsystemHealth::fault;
      }
      else if (imu_stale)
      {
         m_health_summary.imu_health = aeromight_boundaries::SubsystemHealth::degraded;
      }
      else
      {
         m_health_summary.imu_health = aeromight_boundaries::SubsystemHealth::operational;
      }
   }

   void evaluate_barometer_health()
   {
      const auto barometer_health = m_barometer_health_subscriber.get_latest();

      m_health_summary.barometer_operational = barometer_health.data.available;

      const bool barometer_stale = (((m_current_time_ms - barometer_health.timestamp_ms) > m_max_age_barometer_health_ms));

      if (!m_health_summary.barometer_operational)
      {
         m_health_summary.barometer_health = aeromight_boundaries::SubsystemHealth::fault;
      }
      else if (barometer_stale)
      {
         m_health_summary.barometer_health = aeromight_boundaries::SubsystemHealth::degraded;
      }
      else
      {
         m_health_summary.barometer_health = aeromight_boundaries::SubsystemHealth::operational;
      }
   }

   void evaluate_estimation_health()
   {
      const auto estimation_health = m_estimation_health_subscriber.get_latest();

      m_health_summary.estimation_operational = (estimation_health.data.enabled &&
                                                 (estimation_health.data.error.to_ullong() == 0));

      const bool estimation_stale = (((m_current_time_ms - estimation_health.timestamp_ms) > m_max_age_estimation_health_ms));

      if (!m_health_summary.estimation_operational)
      {
         m_health_summary.estimation_health = aeromight_boundaries::SubsystemHealth::fault;
      }
      else if (estimation_stale)
      {
         m_health_summary.estimation_health = aeromight_boundaries::SubsystemHealth::degraded;
      }
      else
      {
         m_health_summary.estimation_health = aeromight_boundaries::SubsystemHealth::operational;
      }
   }

   void evaluate_control_health()
   {
      const auto control_health = m_control_health_subscriber.get_latest();

      m_health_summary.control_operational = control_health.data.enabled;

      const bool control_stale = (((m_current_time_ms - control_health.timestamp_ms) > m_max_age_control_health_ms) ||
                                  (control_health.data.error.to_ullong() != 0));

      if (!m_health_summary.control_operational)
      {
         m_health_summary.control_health = aeromight_boundaries::SubsystemHealth::fault;
      }
      else if (control_stale)
      {
         m_health_summary.control_health = aeromight_boundaries::SubsystemHealth::degraded;
      }
      else
      {
         m_health_summary.control_health = aeromight_boundaries::SubsystemHealth::operational;
      }
   }

   void publish_health_summary()
   {
      m_health_summary.timestamp_ms = m_current_time_ms;
      const bool ok                 = m_queue_sender.send_if_possible(m_health_summary);
      if (!ok)
      {
         m_health_summary.total_queue_failure_count++;
      }
   }

   void publish_battery_status()
   {
      m_battery_status = m_battery.get_status();
      m_battery_status_publisher.update_latest(m_battery_status, m_current_time_ms);
   }

   Battery&                                                             m_battery;
   QueueSender&                                                         m_queue_sender;
   BatteryStatusPublisher&                                              m_battery_status_publisher;
   const boundaries::SharedData<imu::ImuStatus>&                        m_imu_health_subscriber;
   const boundaries::SharedData<barometer::BarometerStatus>&            m_barometer_health_subscriber;
   const boundaries::SharedData<aeromight_boundaries::EstimatorStatus>& m_estimation_health_subscriber;
   const boundaries::SharedData<aeromight_boundaries::ControlStatus>&   m_control_health_subscriber;
   Logger&                                                              m_logger;
   const uint32_t                                                       m_period_in_ms;
   const uint32_t                                                       m_max_age_imu_health_ms;
   const uint32_t                                                       m_max_age_barometer_health_ms;
   const uint32_t                                                       m_max_age_estimation_health_ms;
   const uint32_t                                                       m_max_age_control_health_ms;
   const bool                                                           m_evaluate_barometer_health;
   aeromight_boundaries::HealthSummary                                  m_health_summary{};
   power::battery::BatteryStatus                                        m_battery_status{};
   uint32_t                                                             m_current_time_ms{0};
};

}   // namespace aeromight_health

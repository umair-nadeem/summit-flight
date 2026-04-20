#pragma once

#include "SystemManagerState.hpp"
#include "aeromight_boundaries/HealthSummary.hpp"
#include "aeromight_boundaries/SystemControlSetpoints.hpp"
#include "aeromight_boundaries/SystemState.hpp"
#include "boundaries/SharedData.hpp"
#include "error/error_handler.hpp"
#include "interfaces/IClockSource.hpp"
#include "interfaces/pcb_component/ILed.hpp"
#include "interfaces/rtos/INotifier.hpp"
#include "interfaces/rtos/IQueueReceiver.hpp"
#include "rc/crsf/LinkStats.hpp"

namespace aeromight_system
{

template <interfaces::rtos::IQueueReceiver<aeromight_boundaries::HealthSummary> QueueReceiver,
          interfaces::rtos::INotifier                                           Notifier,
          interfaces::pcb_component::ILed                                       Led,
          interfaces::IClockSource                                              ClockSource,
          typename Logger>
class SystemManagerStateHandler
{
   using SystemStatePublisher             = boundaries::SharedData<aeromight_boundaries::SystemState>;
   using SystemControlSetpointsSubscriber = boundaries::SharedData<aeromight_boundaries::SystemControlSetpoints>;
   using LinkStatsActualsSubscriber       = boundaries::SharedData<rc::crsf::LinkStats>;

public:
   explicit SystemManagerStateHandler(QueueReceiver&                          health_summary_queue_receiver,
                                      Notifier&                               control_start_notifier,
                                      Notifier&                               imu_start_calibration_notifier,
                                      Led&                                    led,
                                      SystemStatePublisher&                   system_state_publisher,
                                      const SystemControlSetpointsSubscriber& system_control_setpoints_subscriber,
                                      const LinkStatsActualsSubscriber&       link_stats_actuals_subscriber,
                                      Logger&                                 logger,
                                      const float                             stick_input_deadband_abs,
                                      const uint8_t                           good_uplink_quality_pct,
                                      const float                             min_good_signal_rssi_dbm,
                                      const uint32_t                          max_age_stale_data_ms,
                                      const uint32_t                          min_state_debounce_duration_ms,
                                      const uint32_t                          timeout_sensors_readiness_ms,
                                      const uint32_t                          timeout_control_readiness_ms)
       : m_health_summary_queue_receiver{health_summary_queue_receiver},
         m_control_start_notifier(control_start_notifier),
         m_imu_start_calibration_notifier(imu_start_calibration_notifier),
         m_led{led},
         m_system_state_publisher(system_state_publisher),
         m_system_control_setpoints_subscriber(system_control_setpoints_subscriber),
         m_link_stats_actuals_subscriber(link_stats_actuals_subscriber),
         m_logger{logger},
         m_stick_input_deadband_abs{stick_input_deadband_abs},
         m_good_uplink_quality_pct{good_uplink_quality_pct},
         m_min_good_signal_rssi_dbm{min_good_signal_rssi_dbm},
         m_max_age_stale_data_ms{max_age_stale_data_ms},
         m_min_state_debounce_duration_ms{min_state_debounce_duration_ms},
         m_timeout_sensors_readiness_ms{timeout_sensors_readiness_ms},
         m_timeout_control_readiness_ms{timeout_control_readiness_ms}
   {
   }

   void get_time()
   {
      m_current_time_ms = ClockSource::now_ms();
   }

   void set_reference_time()
   {
      m_reference_time_ms = ClockSource::now_ms();
   }

   void read_health_summary()
   {
      const auto summary = m_health_summary_queue_receiver.receive_latest();
      if (summary.has_value())
      {
         m_health_summary = summary.value();
      }
   }

   void get_rc_input()
   {
      m_system_control_setpoints = m_system_control_setpoints_subscriber.get_latest();
      m_link_stats_actuals       = m_link_stats_actuals_subscriber.get_latest();
   }

   void publish_system_state()
   {
      m_system_state_publisher.update_latest(m_system_state, m_current_time_ms);
   }

   void start_control()
   {
      m_control_start_notifier.notify();
      m_logger.print("started control");
   }

   void start_imu_calibration()
   {
      m_imu_start_calibration_notifier.notify();
      m_logger.print("start imu calibration");
   }

   void arm_system()
   {
      m_system_state.armed = true;
   }

   void disarm_system()
   {
      m_system_state.armed = false;
   }

   void set_state(const SystemManagerState state)
   {
      m_state = state;

      switch (m_state)
      {
         case SystemManagerState::init:
            // do nothing
            break;

         case SystemManagerState::wait_sensors:
            m_logger.print("entered state->wait_sensors");
            break;

         case SystemManagerState::wait_control:
            m_logger.print("entered state->wait_control");
            break;

         case SystemManagerState::imu_calibration:
            m_logger.print("entered state->imu_calibration");
            break;

         case SystemManagerState::disarming:
            m_logger.print("entered state->disarming");
            break;

         case SystemManagerState::disarmed:
            m_logger.print("entered state->disarmed");
            break;

         case SystemManagerState::arming:
            m_logger.print("entered state->arming");
            break;

         case SystemManagerState::armed:
            m_logger.print("entered state->armed");
            break;

         case SystemManagerState::fault:
            m_logger.print("entered state->fault");
            break;

         default:
            error::stop_operation();
            break;
      }
   }

   void set_calibration_led()
   {
      toggle_status_led(calibration_led_period_ms);
   }

   void set_disarmed_led()
   {
      turn_on_status_led();
   }

   void set_arming_led()
   {
      turn_off_status_led();
   }

   void set_armed_led()
   {
      toggle_status_led(armed_led_period_ms);
   }

   void set_fault_led()
   {
      turn_off_status_led();
   }

   SystemManagerState get_state() const
   {
      return m_state;
   }

   bool health_summary_received() const
   {
      return (m_health_summary.timestamp_ms > 0);
   }

   bool radio_input_received() const
   {
      return ((m_system_control_setpoints.timestamp_ms > 0) && (m_link_stats_actuals.timestamp_ms > 0));
   }

   bool timeout_sensors_readiness() const
   {
      return ((m_current_time_ms - m_reference_time_ms) >= m_timeout_sensors_readiness_ms);
   }

   bool timeout_control_readiness() const
   {
      return ((m_current_time_ms - m_reference_time_ms) >= m_timeout_control_readiness_ms);
   }

   bool is_state_change_persistent() const
   {
      return ((m_current_time_ms - m_reference_time_ms) >= m_min_state_debounce_duration_ms);
   }

   bool sensors_ready() const
   {
      return m_health_summary.imu_operational;
   }

   bool control_ready() const
   {
      return (m_health_summary.estimation_operational && m_health_summary.control_operational);
   }

   bool arm() const
   {
      return m_system_control_setpoints.data.arm;
   }

   bool disarm() const
   {
      return !m_system_control_setpoints.data.arm;
   }

   bool imu_calibration() const
   {
      return (m_system_control_setpoints.data.imu_calibration);
   }

   bool imu_calibration_finished() const
   {
      return (m_health_summary.imu_calibration_finished);
   }

   bool stale_health() const
   {
      return ((m_current_time_ms - m_health_summary.timestamp_ms) >= m_max_age_stale_data_ms);
   }

   bool stale_radio_input() const
   {
      if (((m_current_time_ms - m_system_control_setpoints.timestamp_ms) >= m_max_age_stale_data_ms) ||
          ((m_current_time_ms - m_link_stats_actuals.timestamp_ms) >= m_max_age_stale_data_ms))
      {
         return true;
      }

      return false;
   }

   bool is_health_good() const
   {
      if (!stale_health())
      {
         if (is_imu_operational())
         {
            if (is_estimation_operational())
            {
               if (is_control_operational())
               {
                  return true;
               }
               else
               {
                  m_logger.print("control is not operational");
               }
            }
            else
            {
               m_logger.print("estimation is not operational");
            }
         }
         else
         {
            m_logger.print("imu is not operational");
         }
      }
      else
      {
         m_logger.print("stale health data");
      }

      return false;
   }

   bool is_radio_link_good() const
   {
      if (!stale_radio_input())
      {
         return ((m_link_stats_actuals.data.uplink_quality_pct > m_good_uplink_quality_pct) &&
                 (m_link_stats_actuals.data.uplink_rssi_1_dbm >= m_min_good_signal_rssi_dbm));
      }

      return false;
   }

private:
   // cppcheck-suppress functionStatic
   void turn_on_status_led()
   {
      m_led.turn_on();
   }

   // cppcheck-suppress functionStatic
   void turn_off_status_led()
   {
      m_led.turn_off();
   }

   // cppcheck-suppress functionStatic
   void toggle_status_led(const uint32_t period)
   {
      m_led.toggle(period);
   }

   bool is_imu_operational() const
   {
      return (m_health_summary.imu_health == aeromight_boundaries::SubsystemHealth::operational);
   }

   bool is_estimation_operational() const
   {
      return (m_health_summary.estimation_health == aeromight_boundaries::SubsystemHealth::operational);
   }

   bool is_control_operational() const
   {
      return (m_health_summary.control_health == aeromight_boundaries::SubsystemHealth::operational);
   }

   static constexpr uint32_t calibration_led_period_ms = 150u;
   static constexpr uint32_t armed_led_period_ms       = 400u;

   QueueReceiver&                           m_health_summary_queue_receiver;
   Notifier&                                m_control_start_notifier;
   Notifier&                                m_imu_start_calibration_notifier;
   Led&                                     m_led;
   SystemStatePublisher&                    m_system_state_publisher;
   const SystemControlSetpointsSubscriber&  m_system_control_setpoints_subscriber;
   const LinkStatsActualsSubscriber&        m_link_stats_actuals_subscriber;
   Logger&                                  m_logger;
   const float                              m_stick_input_deadband_abs;
   const uint8_t                            m_good_uplink_quality_pct;
   const float                              m_min_good_signal_rssi_dbm;
   const uint32_t                           m_max_age_stale_data_ms;
   const uint32_t                           m_min_state_debounce_duration_ms;
   const uint32_t                           m_timeout_sensors_readiness_ms;
   const uint32_t                           m_timeout_control_readiness_ms;
   aeromight_boundaries::SystemState        m_system_state{};
   aeromight_boundaries::HealthSummary      m_health_summary{};
   SystemControlSetpointsSubscriber::Sample m_system_control_setpoints{};
   LinkStatsActualsSubscriber::Sample       m_link_stats_actuals{};
   SystemManagerState                       m_state{SystemManagerState::init};
   uint32_t                                 m_current_time_ms{0};
   uint32_t                                 m_reference_time_ms{0};
};

}   // namespace aeromight_system

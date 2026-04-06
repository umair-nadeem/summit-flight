#pragma once

#include "SystemManagerState.hpp"
#include "aeromight_boundaries/HealthSummary.hpp"
#include "aeromight_boundaries/RadioLinkActuals.hpp"
#include "aeromight_boundaries/SystemControlSetpoints.hpp"
#include "aeromight_boundaries/SystemStateInfo.hpp"
#include "boundaries/SharedData.hpp"
#include "error/error_handler.hpp"
#include "interfaces/IClockSource.hpp"
#include "interfaces/pcb_component/ILed.hpp"
#include "interfaces/rtos/INotifier.hpp"
#include "interfaces/rtos/IQueueReceiver.hpp"

namespace aeromight_system
{

template <interfaces::rtos::IQueueReceiver<aeromight_boundaries::HealthSummary> QueueReceiver,
          interfaces::rtos::INotifier                                           Notifier,
          interfaces::pcb_component::ILed                                       Led,
          interfaces::IClockSource                                              ClockSource,
          typename Logger>
class SystemManagerStateHandler
{
   using SystemStateInfoPublisher         = boundaries::SharedData<aeromight_boundaries::SystemStateInfo>;
   using SystemControlSetpointsSubscriber = boundaries::SharedData<aeromight_boundaries::SystemControlSetpoints>;
   using RadioLinkActualsSubscriber       = boundaries::SharedData<aeromight_boundaries::RadioLinkActuals>;

public:
   explicit SystemManagerStateHandler(QueueReceiver&                          health_summary_queue_receiver,
                                      Notifier&                               control_start_notifier,
                                      Notifier&                               imu_start_calibration_notifier,
                                      Led&                                    led,
                                      SystemStateInfoPublisher&               system_state_info_publisher,
                                      const SystemControlSetpointsSubscriber& system_control_setpoints_subscriber,
                                      const RadioLinkActualsSubscriber&       radio_link_actuals_subscriber,
                                      Logger&                                 logger,
                                      const float                             stick_input_deadband_abs,
                                      const float                             min_good_signal_rssi_dbm,
                                      const uint32_t                          max_age_stale_data_ms,
                                      const uint32_t                          min_state_debounce_duration_ms,
                                      const uint32_t                          timeout_sensors_readiness_ms,
                                      const uint32_t                          timeout_control_readiness_ms)
       : m_health_summary_queue_receiver{health_summary_queue_receiver},
         m_control_start_notifier(control_start_notifier),
         m_imu_start_calibration_notifier(imu_start_calibration_notifier),
         m_led{led},
         m_system_state_info_publisher(system_state_info_publisher),
         m_system_control_setpoints_subscriber(system_control_setpoints_subscriber),
         m_radio_link_actuals_subscriber(radio_link_actuals_subscriber),
         m_logger{logger},
         m_stick_input_deadband_abs{stick_input_deadband_abs},
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

   void read_radio_input()
   {
      m_system_control_setpoints = m_system_control_setpoints_subscriber.get_latest();
      m_radio_link_actuals       = m_radio_link_actuals_subscriber.get_latest();
   }

   void publish_system_state_info()
   {
      m_system_state_info_publisher.update_latest(m_system_state_info, m_current_time_ms);
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
      m_system_state_info.armed = true;
   }

   void disarm_system()
   {
      m_system_state_info.armed = false;
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
            turn_off_status_led();
            m_logger.print("entered state->imu_calibration");
            break;

         case SystemManagerState::disarming:
            turn_off_status_led();
            m_logger.print("entered state->disarming");
            break;

         case SystemManagerState::disarmed:
            m_logger.print("entered state->disarmed");
            break;

         case SystemManagerState::arming:
            turn_off_status_led();
            m_logger.print("entered state->arming");
            break;

         case SystemManagerState::armed:
            turn_on_status_led();
            m_logger.print("entered state->armed");
            break;

         case SystemManagerState::fault:
            turn_off_status_led();
            m_logger.print("entered state->fault");
            break;

         default:
            error::stop_operation();
            break;
      }
   }

   void show_disarm_led()
   {
      toggle_status_led();
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
      return ((m_system_control_setpoints.timestamp_ms > 0) && (m_radio_link_actuals.timestamp_ms > 0));
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
      return m_health_summary.all_sensors_ready;
   }

   bool control_ready() const
   {
      return m_health_summary.estimation_ready;
   }

   bool arm() const
   {
      return (m_system_control_setpoints.data.state == aeromight_boundaries::SystemArmedState::arm);
   }

   bool disarm() const
   {
      return (m_system_control_setpoints.data.state == aeromight_boundaries::SystemArmedState::disarm);
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
          ((m_current_time_ms - m_radio_link_actuals.timestamp_ms) >= m_max_age_stale_data_ms))
      {
         return true;
      }

      return false;
   }

   bool is_health_good() const
   {
      if (!stale_health())
      {
         if (m_health_summary.flight_health != aeromight_boundaries::FlightHealthStatus::critical)
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
                     m_logger.print("control failure");
                  }
               }
               else
               {
                  m_logger.print("estimation failure");
               }
            }
            else
            {
               m_logger.print("imu failure");
            }
         }
         else
         {
            m_logger.print("flight health critical");
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
         return (m_radio_link_actuals.data.link_status_ok &&
                 (m_radio_link_actuals.data.link_rssi_dbm >= m_min_good_signal_rssi_dbm));
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

   void toggle_status_led()
   {
      if ((m_current_time_ms - m_status_led_timer) >= led_state_duration)
      {
         if (m_status_led_on)
         {
            m_led.turn_off();
            m_status_led_on    = false;
            m_status_led_timer = m_current_time_ms;
         }
         else
         {
            m_led.turn_on();
            m_status_led_on    = true;
            m_status_led_timer = m_current_time_ms;
         }
      }
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

   static constexpr uint32_t led_state_duration = 750u;

   QueueReceiver&                           m_health_summary_queue_receiver;
   Notifier&                                m_control_start_notifier;
   Notifier&                                m_imu_start_calibration_notifier;
   Led&                                     m_led;
   SystemStateInfoPublisher&                m_system_state_info_publisher;
   const SystemControlSetpointsSubscriber&  m_system_control_setpoints_subscriber;
   const RadioLinkActualsSubscriber&        m_radio_link_actuals_subscriber;
   Logger&                                  m_logger;
   const float                              m_stick_input_deadband_abs;
   const float                              m_min_good_signal_rssi_dbm;
   const uint32_t                           m_max_age_stale_data_ms;
   const uint32_t                           m_min_state_debounce_duration_ms;
   const uint32_t                           m_timeout_sensors_readiness_ms;
   const uint32_t                           m_timeout_control_readiness_ms;
   aeromight_boundaries::SystemStateInfo    m_system_state_info{};
   aeromight_boundaries::HealthSummary      m_health_summary{};
   SystemControlSetpointsSubscriber::Sample m_system_control_setpoints{};
   RadioLinkActualsSubscriber::Sample       m_radio_link_actuals{};
   SystemManagerState                       m_state{SystemManagerState::init};
   uint32_t                                 m_current_time_ms{0};
   uint32_t                                 m_reference_time_ms{0};
   uint32_t                                 m_status_led_timer{0};
   bool                                     m_status_led_on{false};
};

}   // namespace aeromight_system

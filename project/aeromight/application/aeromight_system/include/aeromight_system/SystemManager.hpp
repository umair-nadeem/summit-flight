#pragma once

#include "MainSM.hpp"
#include "SystemManagerStateHandler.hpp"

namespace aeromight_system
{

template <interfaces::rtos::IQueueReceiver<aeromight_boundaries::HealthSummary> QueueReceiver,
          interfaces::rtos::INotifier                                           Notifier,
          interfaces::pcb_component::ILed                                       Led,
          interfaces::IClockSource                                              ClockSource,
          typename Logger>
class SystemManager
{
public:
   explicit SystemManager(QueueReceiver&                                                              health_summary_queue_receiver,
                          Notifier&                                                                   control_start_notifier,
                          Notifier&                                                                   imu_start_calibration_notifier,
                          Led&                                                                        led,
                          boundaries::SharedData<aeromight_boundaries::SystemState>&                  system_state_publisher,
                          const boundaries::SharedData<aeromight_boundaries::SystemControlSetpoints>& system_control_setpoints_subscriber,
                          const boundaries::SharedData<rc::crsf::LinkStats>&                          link_stats_actuals_subscriber,
                          Logger&                                                                     logger,
                          const float                                                                 stick_input_deadband_abs,
                          const uint8_t                                                               good_uplink_quality_pct,
                          const float                                                                 min_good_signal_rssi_dbm,
                          const uint32_t                                                              period_ms,
                          const uint32_t                                                              max_age_stale_data_ms,
                          const uint32_t                                                              min_state_debounce_duration_ms,
                          const uint32_t                                                              timeout_sensors_readiness_ms,
                          const uint32_t                                                              timeout_control_readiness_ms)
       : m_period_ms{period_ms},
         m_state_handler{health_summary_queue_receiver,
                         control_start_notifier,
                         imu_start_calibration_notifier,
                         led,
                         system_state_publisher,
                         system_control_setpoints_subscriber,
                         link_stats_actuals_subscriber,
                         logger,
                         stick_input_deadband_abs,
                         good_uplink_quality_pct,
                         min_good_signal_rssi_dbm,
                         max_age_stale_data_ms,
                         min_state_debounce_duration_ms,
                         timeout_sensors_readiness_ms,
                         timeout_control_readiness_ms}
   {
      logger.enable();
   }

   void run_once()
   {
      m_state_handler.get_time();
      m_state_handler.read_health_summary();
      m_state_handler.get_rc_input();

      m_state_machine.process_event(typename StateMachineDef::EventTick{});

      m_state_handler.publish_system_state();
   }

   SystemManagerState get_state() const
   {
      return m_state_handler.get_state();
   }

   uint32_t get_period_ms() const
   {
      return m_period_ms;
   }

private:
   using StateHandler    = SystemManagerStateHandler<QueueReceiver, Notifier, Led, ClockSource, Logger>;
   using StateMachineDef = SystemManagerStateMachine<StateHandler>;

   const uint32_t                  m_period_ms;
   StateHandler                    m_state_handler;
   boost::sml::sm<StateMachineDef> m_state_machine{m_state_handler};
};
}   // namespace aeromight_system

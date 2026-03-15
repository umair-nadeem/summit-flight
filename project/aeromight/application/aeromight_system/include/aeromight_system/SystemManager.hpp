#pragma once

#include "MainSM.hpp"
#include "SystemManagerStateHandler.hpp"

namespace aeromight_system
{

template <interfaces::rtos::IQueueReceiver<aeromight_boundaries::HealthSummary> QueueReceiver,
          interfaces::rtos::INotifier                                           EstimationNotifier,
          interfaces::pcb_component::ILed                                       Led,
          interfaces::IClockSource                                              ClockSource,
          typename Logger>
class SystemManager
{
public:
   explicit SystemManager(QueueReceiver&                                                              health_summary_queue_receiver,
                          EstimationNotifier&                                                         control_start_notifier,
                          Led&                                                                        led,
                          boundaries::SharedData<aeromight_boundaries::SystemStateInfo>&              system_state_info_storage,
                          const boundaries::SharedData<aeromight_boundaries::SystemControlSetpoints>& system_control_setpoints_storage,
                          const boundaries::SharedData<aeromight_boundaries::RadioLinkActuals>&       radio_link_actuals_storage,
                          Logger&                                                                     logger,
                          const float                                                                 stick_input_deadband_abs,
                          const float                                                                 min_good_signal_rssi_dbm,
                          const uint32_t                                                              period_ms,
                          const uint32_t                                                              max_age_stale_data_ms,
                          const uint32_t                                                              min_state_debounce_duration_ms,
                          const uint32_t                                                              timeout_sensors_readiness_ms,
                          const uint32_t                                                              timeout_control_readiness_ms)
       : m_period_ms{period_ms},
         m_state_handler{health_summary_queue_receiver,
                         control_start_notifier,
                         led,
                         system_state_info_storage,
                         system_control_setpoints_storage,
                         radio_link_actuals_storage,
                         logger,
                         stick_input_deadband_abs,
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
      m_state_handler.read_radio_input();
      m_state_machine.process_event(typename StateMachineDef::EventTick{});
      m_state_handler.publish_system_state_info();
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
   using StateHandler    = SystemManagerStateHandler<QueueReceiver, EstimationNotifier, Led, ClockSource, Logger>;
   using StateMachineDef = SystemManagerStateMachine<StateHandler>;

   const uint32_t                  m_period_ms;
   StateHandler                    m_state_handler;
   boost::sml::sm<StateMachineDef> m_state_machine{m_state_handler};
};
}   // namespace aeromight_system

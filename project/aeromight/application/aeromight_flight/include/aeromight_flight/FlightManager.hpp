#pragma once

#include "FlightManagerStateHandler.hpp"
#include "MainSM.hpp"

namespace aeromight_flight
{

template <interfaces::rtos::IQueueReceiver<aeromight_boundaries::HealthSummary> QueueReceiver,
          interfaces::rtos::INotifier                                           EstimationNotifier,
          interfaces::pcb_component::ILed                                       Led,
          interfaces::IClockSource                                              ClockSource,
          typename Logger>
class FlightManager
{
public:
   explicit FlightManager(QueueReceiver&                                                             health_summary_queue_receiver,
                          EstimationNotifier&                                                        control_start_notifier,
                          Led&                                                                       led,
                          boundaries::SharedData<aeromight_boundaries::FlightControlSetpoints>&      flight_control_setpoints_storage,
                          const boundaries::SharedData<aeromight_boundaries::RadioControlSetpoints>& radio_control_setpoints_storage,
                          const boundaries::SharedData<aeromight_boundaries::RadioLinkStats>&        radio_link_actuals_storage,
                          Logger&                                                                    logger,
                          const float                                                                stick_input_deadband_abs,
                          const float                                                                min_good_signal_rssi_dbm,
                          const float                                                                arming_throttle,
                          const uint32_t                                                             period_ms,
                          const uint32_t                                                             max_age_stale_data_ms,
                          const uint32_t                                                             min_state_debounce_duration_ms,
                          const uint32_t                                                             timeout_sensors_readiness_ms,
                          const uint32_t                                                             timeout_control_readiness_ms,
                          const uint32_t                                                             timeout_auto_land_ms)
       : m_period_ms{period_ms},
         m_state_handler{health_summary_queue_receiver,
                         control_start_notifier,
                         led,
                         flight_control_setpoints_storage,
                         radio_control_setpoints_storage,
                         radio_link_actuals_storage,
                         logger,
                         stick_input_deadband_abs,
                         min_good_signal_rssi_dbm,
                         arming_throttle,
                         max_age_stale_data_ms,
                         min_state_debounce_duration_ms,
                         timeout_sensors_readiness_ms,
                         timeout_control_readiness_ms,
                         timeout_auto_land_ms}
   {
      logger.enable();
   }

   void run_once()
   {
      m_state_handler.get_time();
      m_state_machine.process_event(typename StateMachineDef::EventTick{});
   }

   FlightManagerState get_state() const
   {
      return m_state_handler.get_state();
   }

   uint32_t get_period_ms() const
   {
      return m_period_ms;
   }

private:
   using StateHandler    = FlightManagerStateHandler<QueueReceiver, EstimationNotifier, Led, ClockSource, Logger>;
   using StateMachineDef = FlightManagerStateMachine<StateHandler>;

   const uint32_t                  m_period_ms;
   StateHandler                    m_state_handler;
   boost::sml::sm<StateMachineDef> m_state_machine{m_state_handler};
};
}   // namespace aeromight_flight

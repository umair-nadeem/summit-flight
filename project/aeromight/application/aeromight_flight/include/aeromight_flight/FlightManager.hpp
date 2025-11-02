#pragma once

#include "FlightManagerStateHandler.hpp"
#include "MainSM.hpp"

namespace aeromight_flight
{

template <interfaces::rtos::IQueueReceiver<aeromight_boundaries::HealthSummary> QueueReceiver,
          interfaces::rtos::INotifier                                           EstimationNotifier,
          interfaces::IClockSource                                              ClockSource,
          typename Logger>
class FlightManager
{
public:
   explicit FlightManager(QueueReceiver&                                                              health_summary_queue_receiver,
                          EstimationNotifier&                                                         control_start_notifier,
                          const boundaries::SharedData<aeromight_boundaries::FlightManagerSetpoints>& setpoints,
                          const boundaries::SharedData<aeromight_boundaries::FlightManagerActuals>&   actuals,
                          Logger&                                                                     logger,
                          const float                                                                 stick_input_deadband_abs,
                          const float                                                                 min_good_signal_rssi_dbm,
                          const uint32_t                                                              period_ms,
                          const uint32_t                                                              max_age_stale_data_ms,
                          const uint32_t                                                              min_state_debounce_duration_ms,
                          const uint32_t                                                              timeout_sensors_readiness_ms,
                          const uint32_t                                                              timeout_control_readiness_ms,
                          const uint32_t                                                              timeout_armed_no_flight_ms,
                          const uint32_t                                                              timeout_auto_land_ms)
       : m_period_ms{period_ms},
         m_state_handler{health_summary_queue_receiver,
                         control_start_notifier,
                         setpoints,
                         actuals,
                         logger,
                         stick_input_deadband_abs,
                         min_good_signal_rssi_dbm,
                         max_age_stale_data_ms,
                         min_state_debounce_duration_ms,
                         timeout_sensors_readiness_ms,
                         timeout_control_readiness_ms,
                         timeout_armed_no_flight_ms,
                         timeout_auto_land_ms}
   {
      logger.enable();
   }

   void run_once()
   {
      m_state_machine.process_event(typename StateMachineDef::EventTick{});
   }

   uint32_t get_period_ms() const
   {
      return m_period_ms;
   }

private:
   using StateHandler    = FlightManagerStateHandler<QueueReceiver, EstimationNotifier, ClockSource, Logger>;
   using StateMachineDef = FlightManagerStateMachine<StateHandler>;

   const uint32_t                  m_period_ms;
   StateHandler                    m_state_handler;
   boost::sml::sm<StateMachineDef> m_state_machine{m_state_handler};
};
}   // namespace aeromight_flight

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
   explicit FlightManager(QueueReceiver&      health_summary_queue_receiver,
                          EstimationNotifier& estimation_start_notifier,
                          Logger&             logger,
                          const std::size_t   period_ms,
                          const uint32_t      max_wait_sensors_readiness_ms,
                          const uint32_t      max_wait_estimation_control_readiness_ms)
       : m_period_ms{period_ms},
         m_state_handler{health_summary_queue_receiver,
                         estimation_start_notifier,
                         logger,
                         max_wait_sensors_readiness_ms,
                         max_wait_estimation_control_readiness_ms}
   {
      logger.enable();
   }

   void run_once()
   {
      m_state_machine.process_event(typename StateMachineDef::EventTick{});
   }

   std::size_t get_period_ms() const
   {
      return m_period_ms;
   }

private:
   using StateHandler    = FlightManagerStateHandler<QueueReceiver, EstimationNotifier, ClockSource, Logger>;
   using StateMachineDef = FlightManagerStateMachine<StateHandler>;

   const std::size_t               m_period_ms;
   StateHandler                    m_state_handler;
   boost::sml::sm<StateMachineDef> m_state_machine{m_state_handler};
};
}   // namespace aeromight_flight

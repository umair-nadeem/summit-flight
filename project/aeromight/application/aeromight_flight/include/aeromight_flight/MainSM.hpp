#pragma once

#include <boost/sml.hpp>

namespace aeromight_flight
{

template <typename StateHandler>
struct FlightManagerStateMachine
{

   // leaf states
   static constexpr auto s_init                    = boost::sml::state<class StateInit>;
   static constexpr auto s_wait_sensors            = boost::sml::state<class StateWaitSensors>;
   static constexpr auto s_check_sensors_readiness = boost::sml::state<class StateCheckSensorsReadiness>;
   static constexpr auto s_wait_control            = boost::sml::state<class StateWaitControl>;
   static constexpr auto s_idle                    = boost::sml::state<class StateIdle>;
   static constexpr auto s_manual                  = boost::sml::state<class StateManual>;
   static constexpr auto s_failsafe                = boost::sml::state<class StateFailsafe>;

   // events
   struct EventTick
   {
   };

   auto operator()() const
   {
      using namespace boost::sml;

      // actions
      constexpr auto update_reference_time = [](StateHandler& state)
      {
         state.update_reference_time();
      };

      constexpr auto read_health_summary = [](StateHandler& state)
      {
         state.read_health_summary();
      };

      constexpr auto start_estimation = [](StateHandler& state)
      {
         state.start_estimation();
      };

      // guards
      constexpr auto sensors_ready = [](StateHandler& state)
      {
         return state.sensors_ready();
      };

      constexpr auto timeout_sensors_readiness = [](StateHandler& state)
      {
         return state.timeout_sensors_readiness();
      };

      // events
      static constexpr auto e_tick = event<EventTick>;

      return make_transition_table(
          // From State        | Event           | Guard                           | Action                                                      | To State
          *s_init + e_tick / update_reference_time                    = s_wait_sensors,
          s_wait_sensors + e_tick / read_health_summary               = s_check_sensors_readiness,
          s_check_sensors_readiness[sensors_ready] / start_estimation = s_wait_control,
          s_check_sensors_readiness[timeout_sensors_readiness]        = s_failsafe,
          s_check_sensors_readiness                                   = s_wait_sensors);
   }
};

}   // namespace aeromight_flight

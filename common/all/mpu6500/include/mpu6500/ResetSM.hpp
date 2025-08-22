#pragma once

#include <boost/sml.hpp>

#include "Events.hpp"

namespace mpu6500
{

template <typename StateHandler>
struct ResetStateMachine
{
   // states
   static constexpr auto s_power_reset       = boost::sml::state<class StatePowerReset>;
   static constexpr auto s_power_reset_wait  = boost::sml::state<class StatePoweresetWait>;
   static constexpr auto s_signal_reset      = boost::sml::state<class StateSignalPathReset>;
   static constexpr auto s_signal_reset_wait = boost::sml::state<class StateSignalPathResetWait>;

   auto operator()() const
   {
      using namespace boost::sml;

      // actions
      constexpr auto reset_timer = [](StateHandler& state)
      { state.reset_timer(); };

      constexpr auto tick_timer = [](StateHandler& state)
      { state.tick_timer(); };

      constexpr auto power_reset = [](StateHandler& state)
      { state.power_reset(); };

      constexpr auto signal_path_reset = [](StateHandler& state)
      { state.signal_path_reset(); };

      // guards
      constexpr auto power_reset_wait_over = [](const StateHandler& state)
      { return state.power_reset_wait_over(); };

      constexpr auto signal_reset_wait_over = [](const StateHandler& state)
      { return state.signal_reset_wait_over(); };

      // events
      static constexpr auto e_tick = event<EventTick>;

      // clang-format off
      return make_transition_table(
          // From State       | Event       | Guard                    | Action                           | To State
          *s_power_reset      + e_tick                                 / (reset_timer, power_reset)       = s_power_reset_wait,
          s_power_reset_wait  + e_tick      [!power_reset_wait_over]   / tick_timer,
          s_power_reset_wait  + e_tick      [power_reset_wait_over]                                       = s_signal_reset,

          s_signal_reset      + e_tick                                 / (reset_timer, signal_path_reset) = s_signal_reset_wait,
          s_signal_reset_wait + e_tick      [!signal_reset_wait_over]  / tick_timer,
          s_signal_reset_wait + e_tick      [signal_reset_wait_over]                                      = X
      );
      // clang-format on
   }
};

}   // namespace mpu6500

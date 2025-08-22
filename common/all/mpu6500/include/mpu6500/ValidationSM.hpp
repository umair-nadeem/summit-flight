#pragma once

#include <boost/sml.hpp>

#include "Events.hpp"

namespace mpu6500
{

template <typename StateHandler>
struct ValidationStateMachine
{

   // states
   static constexpr auto s_set_bus      = boost::sml::state<class StateSetBus>;
   static constexpr auto s_wakeup       = boost::sml::state<class StateWakeup>;
   static constexpr auto s_read_id      = boost::sml::state<class StateReadId>;
   static constexpr auto s_wait_receive = boost::sml::state<class StateWaitReceiveDone>;
   static constexpr auto s_verify_id    = boost::sml::state<class StateVerifyId>;

   auto operator()() const
   {
      using namespace boost::sml;

      // actions
      constexpr auto set_bus = [](StateHandler& state)
      { state.set_bus(); };

      constexpr auto set_clock_and_wakeup = [](StateHandler& state)
      { state.set_clock_and_wakeup(); };

      constexpr auto read_id = [](StateHandler& state)
      { state.read_id(); };

      constexpr auto mark_validation_fail = [](StateHandler& state)
      { state.mark_validation_fail(); };

      constexpr auto mark_validation_success = [](StateHandler& state)
      { state.mark_validation_success(); };

      // guards
      constexpr auto id_matched = [](StateHandler& state)
      { return state.id_matched(); };

      // events
      static constexpr auto e_tick         = event<EventTick>;
      static constexpr auto e_receive_done = event<EventReceiveDone>;

      // clang-format off
      return make_transition_table(
          // From State    | Event           | Guard           | Action                 | To State
          *s_set_bus           + e_tick                            / set_bus                = s_wakeup,
          s_wakeup        + e_tick                            / set_clock_and_wakeup       = s_read_id,

          s_read_id        + e_tick                            / read_id                = s_wait_receive,

          s_wait_receive   + e_receive_done                                             = s_verify_id,

          s_verify_id      + e_tick          [!id_matched]     / mark_validation_fail     = X,
          s_verify_id      + e_tick          [id_matched]      / mark_validation_success  = X

          // clock and wakeup
      );
      // clang-format on
   }
};

}   // namespace mpu6500

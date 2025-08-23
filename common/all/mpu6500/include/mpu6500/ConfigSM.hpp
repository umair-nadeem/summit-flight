#pragma once

#include <boost/sml.hpp>

#include "Events.hpp"
#include "Mpu6500Error.hpp"

namespace mpu6500
{

template <typename StateHandler>
struct ConfigStateMachine
{
   // states
   static constexpr auto s_config              = boost::sml::state<class StateConfig>;
   static constexpr auto s_read_config         = boost::sml::state<class StateReadConfig>;
   static constexpr auto s_config_receive_wait = boost::sml::state<class StateConfigReceiveWait>;
   static constexpr auto s_verify_config       = boost::sml::state<class StateVerifyConfig>;

   auto operator()() const
   {
      using namespace boost::sml;

      // actions
      constexpr auto reset_timer = [](StateHandler& state)
      { state.reset_timer(); };

      constexpr auto tick_timer = [](StateHandler& state)
      { state.tick_timer(); };

      constexpr auto write_config_burst = [](StateHandler& state)
      { state.write_config_burst(); };

      constexpr auto read_config_burst = [](StateHandler& state)
      { state.read_config_burst(); };

      constexpr auto store_config = [](StateHandler& state)
      { state.store_config(); };

      constexpr auto mark_config_fail = [](StateHandler& state)
      { state.mark_config_fail(); };

      constexpr auto mark_config_success = [](StateHandler& state)
      { state.mark_config_success(); };

      constexpr auto set_bus_error = [](StateHandler& state)
      { state.set_error(Mpu6500Error::bus_error); };

      // guards
      constexpr auto receive_wait_timeout = [](const StateHandler& state)
      { return state.receive_wait_timeout(); };

      constexpr auto config_matched = [](const StateHandler& state)
      { return state.config_matched(); };

      // events
      static constexpr auto e_tick         = event<EventTick>;
      static constexpr auto e_receive_done = event<EventReceiveDone>;

      // clang-format off
      return make_transition_table(
          // From State            | Event           | Guard                    | Action                               | To State
          *s_config                + e_tick                                     / write_config_burst                   = s_read_config,
          s_read_config            + e_tick                                     / (reset_timer, read_config_burst)     = s_config_receive_wait,

          s_config_receive_wait    + e_receive_done                             / store_config                         = s_verify_config,
          s_config_receive_wait    + e_tick          [!receive_wait_timeout]    / tick_timer,               
          s_config_receive_wait    + e_tick          [receive_wait_timeout]     / (set_bus_error, mark_config_fail)    = X,  

          s_verify_config          + e_tick          [!config_matched]          / mark_config_fail                     = X,
          s_verify_config          + e_tick          [config_matched]           / mark_config_success                  = X

      );
      // clang-format on
   }
};

}   // namespace mpu6500

#pragma once

#include "ConfigSM.hpp"
#include "Mpu6500State.hpp"
#include "ResetSM.hpp"
#include "SelfTestSM.hpp"
#include "ValidationSM.hpp"

namespace mpu6500
{

template <typename StateHandler>
struct MainStateMachine
{
   // composite state machines
   static constexpr auto s_reset      = boost::sml::state<ResetStateMachine<StateHandler>>;
   static constexpr auto s_validation = boost::sml::state<ValidationStateMachine<StateHandler>>;
   static constexpr auto s_self_test  = boost::sml::state<SelfTestStateMachine<StateHandler>>;
   static constexpr auto s_config     = boost::sml::state<ConfigStateMachine<StateHandler>>;

   // leaf states
   static constexpr auto s_stopped     = boost::sml::state<class StateStopped>;
   static constexpr auto s_measurement = boost::sml::state<class StateMeasurement>;
   static constexpr auto s_failure     = boost::sml::state<class StateFailure>;

   auto operator()() const
   {
      using namespace boost::sml;

      // actions
      constexpr auto set_reset_state = [](StateHandler& state)
      { state.set_state(Mpu6500State::reset); };

      constexpr auto set_validation_state = [](StateHandler& state)
      { state.set_state(Mpu6500State::validation); };

      constexpr auto set_self_test_state = [](StateHandler& state)
      { state.set_state(Mpu6500State::self_test); };

      constexpr auto set_config_state = [](StateHandler& state)
      { state.set_state(Mpu6500State::config); };

      constexpr auto set_operational_state = [](StateHandler& state)
      { state.set_state(Mpu6500State::operational); };

      constexpr auto set_failure_state = [](StateHandler& state)
      { state.set_state(Mpu6500State::failure); };

      constexpr auto set_stopped_state = [](StateHandler& state)
      { state.set_state(Mpu6500State::stopped); };

      // guards
      constexpr auto validation_successful = [](StateHandler& state)
      { return state.validation_successful(); };

      constexpr auto self_test_successful = [](StateHandler& state)
      { return state.self_test_successful(); };

      constexpr auto config_successful = [](StateHandler& state)
      { return state.config_successful(); };

      // events
      static constexpr auto e_start = event<EventStart>;
      static constexpr auto e_stop  = event<EventStop>;
      // static constexpr auto e_tick  = event<EventTick>;

      // clang-format off
      return make_transition_table(
          // From State  | Event           | Guard                   | Action                  | To State
          *s_stopped        + e_start                                  / set_reset_state            = s_reset,

          s_reset                                                     / set_validation_state           = s_validation,

          s_validation                     [!validation_successful]     / set_failure_state            = s_failure,
          s_validation                     [validation_successful]     / set_self_test_state            = s_self_test,

          s_self_test                      [!self_test_successful]     / set_failure_state            = s_failure,
          s_self_test                      [self_test_successful]      / set_config_state              = s_config,

          s_config                         [!config_successful]       / set_failure_state            = s_failure,
          s_config                         [config_successful]        / set_operational_state         = s_measurement,

          s_reset           + e_stop                                     / set_stopped_state              = s_stopped,
          s_validation      + e_stop                                   / set_stopped_state               = s_stopped,
          s_self_test        + e_stop                                  / set_stopped_state               = s_stopped,
          s_config          + e_stop                                    / set_stopped_state               = s_stopped,
          s_measurement      + e_stop                                    / set_stopped_state               = s_stopped
         );
      // clang-format on
   }
};

}   // namespace mpu6500

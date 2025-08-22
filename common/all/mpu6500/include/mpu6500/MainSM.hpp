#pragma once

#include <boost/sml.hpp>

#include "Events.hpp"
#include "Mpu6500State.hpp"
#include "ResetSM.hpp"
#include "ValidationSM.hpp"

namespace mpu6500
{

template <typename StateHandler>
struct MainStateMachine
{
   // composite states
   static constexpr auto s_resetting  = boost::sml::state<ResetStateMachine<StateHandler>>;
   static constexpr auto s_validation = boost::sml::state<ValidationStateMachine<StateHandler>>;

   // leaf states
   static constexpr auto s_stopped = boost::sml::state<class StateStopped>;
   static constexpr auto s_temp    = boost::sml::state<class StateTemp>;

   // events

   auto operator()() const
   {
      using namespace boost::sml;

      constexpr auto set_init_state = [](StateHandler& state)
      { state.set_state(Mpu6500State::init); };

      static constexpr auto e_start = event<EventStart>;
      static constexpr auto e_stop  = event<EventStop>;
      static constexpr auto e_tick  = event<EventTick>;

      // clang-format off
      return make_transition_table(
          // From State  | Event       | Guard                | Action           | To State
          *s_stopped        + e_start                   / set_init_state            = s_resetting,
          s_resetting         + e_tick                                           = s_validation,
          s_resetting                                                             = s_temp,

          s_resetting         + e_stop                                                = s_stopped,
          s_validation        + e_stop                                                = s_stopped
         );
      // clang-format on
   }
};

}   // namespace mpu6500

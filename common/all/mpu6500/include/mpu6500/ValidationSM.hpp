#pragma once

#include <boost/sml.hpp>

#include "Events.hpp"
#include "imu_sensor/ImuSensorError.hpp"

namespace mpu6500
{

template <typename StateHandler>
struct ValidationStateMachine
{
   // states
   static constexpr auto s_disable_i2c     = boost::sml::state<class StateDisableI2c>;
   static constexpr auto s_wakeup          = boost::sml::state<class StateWakeup>;
   static constexpr auto s_read_id         = boost::sml::state<class StateReadId>;
   static constexpr auto s_id_receive_wait = boost::sml::state<class StateIdReceiveWait>;
   static constexpr auto s_verify_id       = boost::sml::state<class StateVerifyId>;

   auto operator()() const
   {
      using namespace boost::sml;

      // actions
      constexpr auto reset_timer = [](StateHandler& state)
      { state.reset_timer(); };

      constexpr auto tick_timer = [](StateHandler& state)
      { state.tick_timer(); };

      constexpr auto disable_i2c = [](StateHandler& state)
      { state.disable_i2c(); };

      constexpr auto set_clock_and_wakeup = [](StateHandler& state)
      { state.set_clock_and_wakeup(); };

      constexpr auto read_id = [](StateHandler& state)
      { state.read_id(); };

      constexpr auto store_id = [](StateHandler& state)
      { state.store_id(); };

      constexpr auto set_bus_error = [](StateHandler& state)
      { state.set_error(imu_sensor::ImuSensorError::bus_error); };

      constexpr auto mark_validation_fail = [](StateHandler& state)
      { state.mark_validation_fail(); };

      constexpr auto mark_validation_success = [](StateHandler& state)
      { state.mark_validation_success(); };

      // guards
      constexpr auto receive_wait_timeout = [](const StateHandler& state)
      { return state.receive_wait_timeout(); };

      constexpr auto id_matched = [](const StateHandler& state)
      { return state.id_matched(); };

      // events
      static constexpr auto e_tick         = event<EventTick>;
      static constexpr auto e_receive_done = event<EventReceiveDone>;

      // clang-format off
      return make_transition_table(
          // From State       | Event           | Guard                   | Action                                 | To State
          *s_disable_i2c      + e_tick                                    / disable_i2c                            = s_wakeup,
          s_wakeup            + e_tick                                    / set_clock_and_wakeup                   = s_read_id,

          s_read_id           + e_tick                                    / (reset_timer, read_id)                 = s_id_receive_wait,

          s_id_receive_wait   + e_receive_done                            / store_id                               = s_verify_id,
          s_id_receive_wait   + e_tick          [!receive_wait_timeout]   / tick_timer,               
          s_id_receive_wait   + e_tick          [receive_wait_timeout]    / (set_bus_error, mark_validation_fail)  = X,               

          s_verify_id         + e_tick          [!id_matched]             / mark_validation_fail                   = X,
          s_verify_id         + e_tick          [id_matched]              / mark_validation_success                = X

          // clock and wakeup
      );
      // clang-format on
   }
};

}   // namespace mpu6500

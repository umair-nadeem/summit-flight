#pragma once

#include <boost/sml.hpp>

#include "Events.hpp"
#include "barometer_sensor/BarometerSensorError.hpp"

namespace bmp390
{

template <typename StateHandler>
struct SetupStateMachine
{
   // states
   static constexpr auto s_idle         = boost::sml::state<class StateIdle>;
   static constexpr auto s_read_id      = boost::sml::state<class StateReadId>;
   static constexpr auto s_read_id_wait = boost::sml::state<class StateInitReadIdWait>;
   static constexpr auto s_verify_id    = boost::sml::state<class StateVerifyId>;

   static constexpr auto s_oversampling = boost::sml::state<class StateOversampling>;
   static constexpr auto s_data_rate    = boost::sml::state<class StateDataRate>;
   static constexpr auto s_iir_config   = boost::sml::state<class StateIirConfig>;
   static constexpr auto s_sensor_mode  = boost::sml::state<class StateSensorMode>;

   static constexpr auto s_read_config      = boost::sml::state<class StateReadConfig>;
   static constexpr auto s_read_config_wait = boost::sml::state<class StateReadConfigWait>;
   static constexpr auto s_verify_config    = boost::sml::state<class StateVerifyConfig>;
   static constexpr auto s_bus_error        = boost::sml::state<class StateBusError>;

   auto operator()() const
   {
      using namespace boost::sml;

      // actions
      static constexpr auto reset_timer = [](StateHandler& state)
      { state.reset_timer(); };

      static constexpr auto tick_timer = [](StateHandler& state)
      { state.tick_timer(); };

      static constexpr auto soft_reset = [](StateHandler& state)
      { state.soft_reset(); };

      static constexpr auto read_id = [](StateHandler& state)
      { state.read_id(); };

      static constexpr auto set_normal_mode = [](StateHandler& state)
      { state.set_normal_mode(); };

      static constexpr auto set_oversampling = [](StateHandler& state)
      { state.set_oversampling(); };

      static constexpr auto set_data_rate = [](StateHandler& state)
      { state.set_data_rate(); };

      static constexpr auto write_irr_config = [](StateHandler& state)
      { state.write_irr_config(); };

      static constexpr auto read_config_burst = [](StateHandler& state)
      { state.read_config_burst(); };

      static constexpr auto store_id = [](StateHandler& state)
      { state.store_id(); };

      static constexpr auto store_config = [](StateHandler& state)
      { state.store_config(); };

      constexpr auto mark_setup_fail = [](StateHandler& state)
      { state.mark_setup_fail(); };

      constexpr auto mark_setup_success = [](StateHandler& state)
      { state.mark_setup_success(); };

      static constexpr auto set_bus_error = [](StateHandler& state)
      { state.set_error(barometer_sensor::BarometerSensorError::bus_error); };

      static constexpr auto set_id_mismatch_error = [](StateHandler& state)
      { state.set_error(barometer_sensor::BarometerSensorError::id_mismatch_error); };

      static constexpr auto set_config_mismatch_error = [](StateHandler& state)
      { state.set_error(barometer_sensor::BarometerSensorError::config_mismatch_error); };

      // guards
      static constexpr auto receive_wait_timeout = [](const StateHandler& state)
      { return state.receive_wait_timeout(); };

      static constexpr auto transfer_error = [](const StateHandler& state)
      { return state.transfer_error(); };

      constexpr auto id_matched = [](const StateHandler& state)
      { return state.id_matched(); };

      constexpr auto config_matched = [](const StateHandler& state)
      { return state.config_matched(); };

      // events
      static constexpr auto e_tick         = event<EventTick>;
      static constexpr auto e_receive_done = event<EventReceiveDone>;

      // clang-format off
      return make_transition_table(
          // From State      | Event          | Guard                   | Action                                        | To State
          *s_idle            + e_tick                                   / soft_reset                                    = s_read_id,

          s_read_id          + e_tick         [transfer_error]                                                          = s_bus_error,
          s_read_id          + e_tick         [!transfer_error]         / (reset_timer, read_id)                        = s_read_id_wait,

          s_read_id_wait     + e_receive_done                           / store_id                                      = s_verify_id,
          s_read_id_wait     + e_tick         [transfer_error]                                                          = s_bus_error,
          s_read_id_wait     + e_tick         [!receive_wait_timeout]   / tick_timer,
          s_read_id_wait     + e_tick         [receive_wait_timeout]                                                    = s_bus_error,

          s_verify_id                         [!id_matched]             / (set_id_mismatch_error, mark_setup_fail)      = X,
          s_verify_id                         [id_matched]                                                              = s_oversampling,

          s_oversampling     + e_tick                                   / set_oversampling                              = s_data_rate,
          s_data_rate        + e_tick                                   / set_data_rate                                 = s_iir_config,
          s_iir_config       + e_tick                                   / write_irr_config                              = s_sensor_mode,

          s_sensor_mode      + e_tick         [transfer_error]                                                          = s_bus_error,
          s_sensor_mode      + e_tick         [!transfer_error]         / set_normal_mode                               = s_read_config,

          s_read_config      + e_tick         [transfer_error]                                                          = s_bus_error,
          s_read_config      + e_tick         [!transfer_error]         / (reset_timer, read_config_burst)              = s_read_config_wait,

          s_read_config_wait + e_receive_done                           / store_config                                  = s_verify_config,
          s_read_config_wait + e_tick         [transfer_error]                                                          = s_bus_error,
          s_read_config_wait + e_tick         [!receive_wait_timeout]   / tick_timer,
          s_read_config_wait + e_tick         [receive_wait_timeout]                                                    = s_bus_error,

          s_verify_config                     [!config_matched]         / (set_config_mismatch_error, mark_setup_fail)  = X,
          s_verify_config                     [config_matched]          / mark_setup_success                            = X,

          s_bus_error                                                   / (set_bus_error, mark_setup_fail)              = X
         
      );
      // clang-format on
   }
};

}   // namespace bmp390

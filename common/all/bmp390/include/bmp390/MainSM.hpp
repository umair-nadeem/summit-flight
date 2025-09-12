#pragma once

#include "SetupSM.hpp"

namespace bmp390
{

template <typename StateHandler>
struct Bmp390MainStateMachine
{
   // wrappers for setup substate machine
   template <typename H>
   struct InitSetupSM : SetupStateMachine<H>
   {
   };

   template <typename H>
   struct RecoverySetupSM : SetupStateMachine<H>
   {
   };

   // composite state machines
   static constexpr auto s_init_setup     = boost::sml::state<InitSetupSM<StateHandler>>;
   static constexpr auto s_recovery_setup = boost::sml::state<RecoverySetupSM<StateHandler>>;

   // leaf states
   static constexpr auto s_stopped           = boost::sml::state<class StateStopped>;
   static constexpr auto s_read_coeff        = boost::sml::state<class StateReadCoefficients>;
   static constexpr auto s_read_coeff_wait   = boost::sml::state<class StateReadCoefficientsWait>;
   static constexpr auto s_measurement       = boost::sml::state<class StateMeasurement>;
   static constexpr auto s_data_read_wait    = boost::sml::state<class StateDataReadWait>;
   static constexpr auto s_data_verification = boost::sml::state<class StateDataVerification>;
   static constexpr auto s_data_read_fail    = boost::sml::state<class StateDataReadFailure>;
   static constexpr auto s_recovery          = boost::sml::state<class StateRecovery>;
   static constexpr auto s_failure           = boost::sml::state<class StateFailure>;

   auto operator()() const
   {
      using namespace boost::sml;

      // actions
      static constexpr auto reset_timer = [](StateHandler& state)
      { state.reset_timer(); };

      static constexpr auto tick_timer = [](StateHandler& state)
      { state.tick_timer(); };

      constexpr auto reset_read_failures = [](StateHandler& state)
      { state.reset_read_failures(); };

      constexpr auto reset_recovery_attempts = [](StateHandler& state)
      { state.reset_recovery_attempts(); };

      constexpr auto count_read_failure = [](StateHandler& state)
      { state.count_read_failure(); };

      constexpr auto count_recovery_attempt = [](StateHandler& state)
      { state.count_recovery_attempt(); };

      static constexpr auto read_coefficients = [](StateHandler& state)
      { state.read_coefficients(); };

      static constexpr auto read_data = [](StateHandler& state)
      { state.read_data(); };

      static constexpr auto store_coefficients = [](StateHandler& state)
      { state.store_coefficients(); };

      static constexpr auto process_error_register = [](StateHandler& state)
      { state.process_error_register(); };

      static constexpr auto convert_raw_data = [](StateHandler& state)
      { state.convert_raw_data(); };

      constexpr auto publish_data = [](StateHandler& state)
      { state.publish_data(); };

      constexpr auto publish_health = [](StateHandler& state)
      { state.publish_health(); };

      constexpr auto reset_data = [](StateHandler& state)
      { state.reset_data(); };

      static constexpr auto set_setup_state = [](StateHandler& state)
      { state.set_state(barometer_sensor::BarometerSensorState::setup); };

      static constexpr auto set_read_coefficients_state = [](StateHandler& state)
      { state.set_state(barometer_sensor::BarometerSensorState::read_coefficients); };

      static constexpr auto set_operational_state = [](StateHandler& state)
      { state.set_state(barometer_sensor::BarometerSensorState::operational); };

      static constexpr auto set_recovery_state = [](StateHandler& state)
      { state.set_state(barometer_sensor::BarometerSensorState::recovery); };

      static constexpr auto set_failure_state = [](StateHandler& state)
      { state.set_state(barometer_sensor::BarometerSensorState::failure); };

      static constexpr auto set_stopped_state = [](StateHandler& state)
      { state.set_state(barometer_sensor::BarometerSensorState::stopped); };

      static constexpr auto set_bus_error = [](StateHandler& state)
      { state.set_error(barometer_sensor::BarometerSensorError::bus_error); };

      static constexpr auto set_coefficients_pattern_error = [](StateHandler& state)
      { state.set_error(barometer_sensor::BarometerSensorError::coefficients_pattern_error); };

      static constexpr auto set_sensor_error = [](StateHandler& state)
      { state.set_error(barometer_sensor::BarometerSensorError::sensor_error); };

      static constexpr auto set_data_pattern_error = [](StateHandler& state)
      { state.set_error(barometer_sensor::BarometerSensorError::data_pattern_error); };

      static constexpr auto set_out_of_range_data_error = [](StateHandler& state)
      { state.set_error(barometer_sensor::BarometerSensorError::out_of_range_data_error); };

      // guards
      static constexpr auto transfer_error = [](const StateHandler& state)
      { return state.transfer_error(); };

      constexpr auto setup_successful = [](StateHandler& state)
      { return state.setup_successful(); };

      constexpr auto is_coefficients_pattern_ok = [](const StateHandler& state)
      { return state.is_coefficients_pattern_ok(); };

      constexpr auto is_data_pattern_ok = [](const StateHandler& state)
      { return state.is_data_pattern_ok(); };

      constexpr auto sensor_error_reported = [](const StateHandler& state)
      { return state.sensor_error_reported(); };

      constexpr auto is_data_valid = [](const StateHandler& state)
      { return state.is_data_valid(); };

      static constexpr auto receive_wait_timeout = [](StateHandler& state)
      { return state.receive_wait_timeout(); };

      constexpr auto read_failures_below_limit = [](const StateHandler& state)
      { return state.read_failures_below_limit(); };

      constexpr auto recovery_attempts_below_limit = [](const StateHandler& state)
      { return state.recovery_attempts_below_limit(); };

      // events
      static constexpr auto e_start        = event<EventStart>;
      static constexpr auto e_stop         = event<EventStop>;
      static constexpr auto e_tick         = event<EventTick>;
      static constexpr auto e_receive_done = event<EventReceiveDone>;

      return make_transition_table(
          // From State  | Event   | Guard                    | Action                                  | To State
          *s_stopped + e_start / set_setup_state = s_init_setup,

          // setup
          s_init_setup[setup_successful] / (set_read_coefficients_state, publish_health) = s_read_coeff,
          s_init_setup[!setup_successful]                                                = s_failure,

          // read coefficients
          s_read_coeff + e_tick / (reset_timer, read_coefficients)                                                                     = s_read_coeff_wait,
          s_read_coeff_wait + e_receive_done[is_coefficients_pattern_ok] / (store_coefficients, set_operational_state, publish_health) = s_measurement,
          s_read_coeff_wait + e_receive_done[!is_coefficients_pattern_ok] / set_coefficients_pattern_error                             = s_failure,
          s_read_coeff_wait + e_tick[transfer_error] / set_bus_error                                                                   = s_failure,
          s_read_coeff_wait + e_tick[!receive_wait_timeout] / tick_timer,
          s_read_coeff_wait + e_tick[receive_wait_timeout] / set_bus_error = s_failure,

          // operational
          s_measurement + e_tick / (reset_timer, read_data) = s_data_read_wait,

          s_data_read_wait + e_receive_done[is_data_pattern_ok] / (process_error_register, convert_raw_data)    = s_data_verification,
          s_data_read_wait + e_receive_done[!is_data_pattern_ok] / (set_data_pattern_error, count_read_failure) = s_data_read_fail,
          s_data_read_wait + e_tick[!receive_wait_timeout] / tick_timer,
          s_data_read_wait + e_tick[receive_wait_timeout] / (set_bus_error, count_read_failure) = s_data_read_fail,

          s_data_verification[sensor_error_reported] / (set_sensor_error, count_read_failure)      = s_data_read_fail,
          s_data_verification[is_data_valid] / (reset_read_failures, publish_data, publish_health) = s_measurement,
          s_data_verification[!is_data_valid] / (set_out_of_range_data_error, count_read_failure)  = s_data_read_fail,

          s_data_read_fail[read_failures_below_limit]                                         = s_measurement,
          s_data_read_fail[!read_failures_below_limit] / (set_recovery_state, publish_health) = s_recovery,

          // soft recovery re-orchestrates setup composite SM
          s_recovery + e_tick / count_recovery_attempt = s_recovery_setup,

          s_recovery_setup[setup_successful] / (reset_recovery_attempts, set_operational_state, publish_health) = s_measurement,
          s_recovery_setup[!setup_successful && recovery_attempts_below_limit] / publish_health                 = s_recovery,
          s_recovery_setup[!setup_successful && !recovery_attempts_below_limit]                                 = s_failure,

          s_failure + boost::sml::on_entry<_> / (set_failure_state, publish_health, reset_data),

          s_init_setup + e_stop / set_stopped_state      = s_stopped,
          s_recovery_setup + e_stop / set_stopped_state  = s_stopped,
          s_read_coeff + e_stop / set_stopped_state      = s_stopped,
          s_read_coeff_wait + e_stop / set_stopped_state = s_stopped,
          s_measurement + e_stop / set_stopped_state     = s_stopped,
          s_data_read_wait + e_stop / set_stopped_state  = s_stopped,
          s_recovery + e_stop / set_stopped_state        = s_stopped);
   }
};

}   // namespace bmp390

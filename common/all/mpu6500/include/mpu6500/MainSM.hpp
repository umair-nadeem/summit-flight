#pragma once

#include "ConfigSM.hpp"
#include "ResetSM.hpp"
#include "SelfTestSM.hpp"
#include "ValidationSM.hpp"
#include "imu_sensor/ImuSensorState.hpp"

namespace mpu6500
{

template <typename StateHandler>
struct MainStateMachine
{
   // wrappers for reset Substate machine
   template <typename H>
   struct InitResetSM : ResetStateMachine<H>
   {
   };
   template <typename H>
   struct HardRecoveryResetSM : ResetStateMachine<H>
   {
   };

   // wrappers for validation Substate machine
   template <typename H>
   struct InitValidationSM : ValidationStateMachine<H>
   {
   };
   template <typename H>
   struct SoftRecoveryValidationSM : ValidationStateMachine<H>
   {
   };
   template <typename H>
   struct HardRecoveryValidationSM : ValidationStateMachine<H>
   {
   };

   // wrappers for config Substate machine
   template <typename H>
   struct InitConfigSM : ConfigStateMachine<H>
   {
   };
   template <typename H>
   struct SoftRecoveryConfigSM : ConfigStateMachine<H>
   {
   };
   template <typename H>
   struct HardRecoveryConfigSM : ConfigStateMachine<H>
   {
   };

   // composite state machines
   static constexpr auto s_self_test = boost::sml::state<SelfTestStateMachine<StateHandler>>;

   static constexpr auto s_init_reset = boost::sml::state<InitResetSM<StateHandler>>;
   static constexpr auto s_hard_reset = boost::sml::state<HardRecoveryResetSM<StateHandler>>;

   static constexpr auto s_init_validation = boost::sml::state<InitValidationSM<StateHandler>>;
   static constexpr auto s_soft_validation = boost::sml::state<SoftRecoveryValidationSM<StateHandler>>;
   static constexpr auto s_hard_validation = boost::sml::state<HardRecoveryValidationSM<StateHandler>>;

   static constexpr auto s_init_config = boost::sml::state<InitConfigSM<StateHandler>>;
   static constexpr auto s_soft_config = boost::sml::state<SoftRecoveryConfigSM<StateHandler>>;
   static constexpr auto s_hard_config = boost::sml::state<HardRecoveryConfigSM<StateHandler>>;

   // leaf states
   static constexpr auto s_stopped           = boost::sml::state<class StateStopped>;
   static constexpr auto s_measurement       = boost::sml::state<class StateMeasurement>;
   static constexpr auto s_data_read_wait    = boost::sml::state<class StateDataReadWait>;
   static constexpr auto s_data_verification = boost::sml::state<class StateDataVerification>;
   static constexpr auto s_data_read_fail    = boost::sml::state<class StateDataReadFailure>;
   static constexpr auto s_soft_recovery     = boost::sml::state<class StateSoftRecovery>;
   static constexpr auto s_hard_recovery     = boost::sml::state<class StateHardRecovery>;
   static constexpr auto s_failure           = boost::sml::state<class StateFailure>;

   auto operator()() const
   {
      using namespace boost::sml;

      // actions
      constexpr auto reset_timer = [](StateHandler& state)
      { state.reset_timer(); };

      constexpr auto tick_timer = [](StateHandler& state)
      { state.tick_timer(); };

      constexpr auto reset_read_failures = [](StateHandler& state)
      { state.reset_read_failures(); };

      constexpr auto count_read_failure = [](StateHandler& state)
      { state.count_read_failure(); };

      constexpr auto read_data = [](StateHandler& state)
      { state.read_data(); };

      constexpr auto convert_raw_data = [](StateHandler& state)
      { state.convert_raw_data(); };

      constexpr auto publish_data = [](StateHandler& state)
      { state.publish_data(); };

      constexpr auto publish_health = [](StateHandler& state)
      { state.publish_health(); };

      constexpr auto reset_data = [](StateHandler& state)
      { state.reset_data(); };

      constexpr auto set_reset_state = [](StateHandler& state)
      { state.set_state(imu_sensor::ImuSensorState::reset); };

      constexpr auto set_validation_state = [](StateHandler& state)
      { state.set_state(imu_sensor::ImuSensorState::validation); };

      constexpr auto set_self_test_state = [](StateHandler& state)
      { state.set_state(imu_sensor::ImuSensorState::self_test); };

      constexpr auto set_config_state = [](StateHandler& state)
      { state.set_state(imu_sensor::ImuSensorState::config); };

      constexpr auto set_operational_state = [](StateHandler& state)
      { state.set_state(imu_sensor::ImuSensorState::operational); };

      constexpr auto set_soft_recovery_state = [](StateHandler& state)
      { state.set_state(imu_sensor::ImuSensorState::soft_recovery); };

      constexpr auto set_hard_recovery_state = [](StateHandler& state)
      { state.set_state(imu_sensor::ImuSensorState::hard_recovery); };

      constexpr auto set_failure_state = [](StateHandler& state)
      { state.set_state(imu_sensor::ImuSensorState::failure); };

      constexpr auto set_stopped_state = [](StateHandler& state)
      { state.set_state(imu_sensor::ImuSensorState::stopped); };

      constexpr auto set_bus_error = [](StateHandler& state)
      { state.set_error(imu_sensor::ImuSensorError::bus_error); };

      constexpr auto set_sensor_error = [](StateHandler& state)
      { state.set_error(imu_sensor::ImuSensorError::sensor_error); };

      constexpr auto set_data_error = [](StateHandler& state)
      { state.set_error(imu_sensor::ImuSensorError::data_error); };

      // guards
      constexpr auto validation_successful = [](StateHandler& state)
      { return state.validation_successful(); };

      constexpr auto self_test_successful = [](StateHandler& state)
      { return state.self_test_successful(); };

      constexpr auto config_successful = [](StateHandler& state)
      { return state.config_successful(); };

      constexpr auto is_buffer_non_zero = [](const StateHandler& state)
      { return state.is_buffer_non_zero(); };

      constexpr auto is_data_valid = [](const StateHandler& state)
      { return state.is_data_valid(); };

      constexpr auto receive_wait_timeout = [](const StateHandler& state)
      { return state.receive_wait_timeout(); };

      constexpr auto read_failures_below_limit = [](const StateHandler& state)
      { return state.read_failures_below_limit(); };

      // events
      static constexpr auto e_start        = event<EventStart>;
      static constexpr auto e_stop         = event<EventStop>;
      static constexpr auto e_tick         = event<EventTick>;
      static constexpr auto e_receive_done = event<EventReceiveDone>;

      // clang-format off
      return make_transition_table(
          // From State          | Event          | Guard                     | Action                                         | To State
          // init orchestrates reset, valiation, and config composite SMs to bring sensor up
          *s_stopped             + e_start                                    / set_reset_state                                = s_init_reset,

          s_init_reset                                                        / set_validation_state                           = s_init_validation,

          s_init_validation                       [validation_successful]     / set_self_test_state                            = s_self_test,
          s_init_validation                       [!validation_successful]                                                     = s_failure,

          s_self_test                             [self_test_successful]      / set_config_state                               = s_init_config,
          s_self_test                             [!self_test_successful]                                                      = s_failure,

          s_init_config                           [config_successful]         / (set_operational_state, publish_health)        = s_measurement,
          s_init_config                           [!config_successful]                                                         = s_failure,

          // operational
          s_measurement         + e_tick                                       / (reset_timer, read_data)                      = s_data_read_wait,

          s_data_read_wait      + e_receive_done  [is_buffer_non_zero]         / convert_raw_data                              = s_data_verification,
          s_data_read_wait      + e_receive_done  [!is_buffer_non_zero]        / (set_sensor_error, count_read_failure)        = s_data_read_fail,
          s_data_read_wait      + e_tick          [!receive_wait_timeout]      / tick_timer,
          s_data_read_wait      + e_tick          [receive_wait_timeout]       / (set_bus_error, count_read_failure)           = s_data_read_fail,

          s_data_verification                     [is_data_valid]              / (reset_read_failures, publish_data)           = s_measurement,
          s_data_verification                     [!is_data_valid]             / (set_data_error, count_read_failure)          = s_data_read_fail,

          s_data_read_fail                        [read_failures_below_limit]                                                  = s_measurement,
          s_data_read_fail                        [!read_failures_below_limit] / (set_soft_recovery_state, publish_health)     = s_soft_recovery,

          // soft recovery re-orchestrates validation and config composite SMs
          s_soft_recovery       + e_tick                                                                                       = s_soft_validation,

          s_soft_validation                       [validation_successful]                                                      = s_soft_config,
          s_soft_validation                       [!validation_successful]                                                     = s_hard_recovery,

          s_soft_config                           [config_successful]         / (set_operational_state, publish_health)        = s_measurement,
          s_soft_config                           [!config_successful]        / (set_hard_recovery_state, publish_health)      = s_hard_recovery,

          // hard recovery re-orchestrates reset, validation and config composite SMs
          s_hard_recovery       + e_tick                                                                                       = s_hard_reset,

          s_hard_reset                                                                                                         = s_hard_validation,

          s_hard_validation                       [validation_successful]                                                      = s_hard_config,
          s_hard_validation                       [!validation_successful]                                                     = s_failure,

          s_hard_config                           [config_successful]         / (set_operational_state, publish_health)        = s_measurement,
          s_hard_config                           [!config_successful]                                                         = s_failure,

          // failure
          s_failure             + boost::sml::on_entry<_>                     / (set_failure_state, publish_health, reset_data),

          s_self_test           + e_stop                                      / set_stopped_state                              = s_stopped,
          s_init_reset          + e_stop                                      / set_stopped_state                              = s_stopped,
          s_hard_reset          + e_stop                                      / set_stopped_state                              = s_stopped,
          s_init_validation     + e_stop                                      / set_stopped_state                              = s_stopped,
          s_soft_validation     + e_stop                                      / set_stopped_state                              = s_stopped,
          s_hard_validation     + e_stop                                      / set_stopped_state                              = s_stopped,
          s_init_config         + e_stop                                      / set_stopped_state                              = s_stopped,
          s_soft_config         + e_stop                                      / set_stopped_state                              = s_stopped,
          s_hard_config         + e_stop                                      / set_stopped_state                              = s_stopped,
          s_measurement         + e_stop                                      / set_stopped_state                              = s_stopped,
          s_data_read_wait      + e_stop                                      / set_stopped_state                              = s_stopped,
          s_data_verification   + e_stop                                      / set_stopped_state                              = s_stopped,
          s_data_read_fail      + e_stop                                      / set_stopped_state                              = s_stopped,
          s_soft_recovery       + e_stop                                      / set_stopped_state                              = s_stopped,
          s_hard_recovery       + e_stop                                      / set_stopped_state                              = s_stopped);
   }
   // clang-format on
};

}   // namespace mpu6500

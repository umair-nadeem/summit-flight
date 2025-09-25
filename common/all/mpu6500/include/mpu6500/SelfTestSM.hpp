#pragma once

#include <boost/sml.hpp>

#include "Events.hpp"
#include "imu_sensor/ImuSensorError.hpp"

namespace mpu6500
{

template <typename StateHandler>
struct SelfTestStateMachine
{
   // states
   static constexpr auto s_reset_stats         = boost::sml::state<class StateResetStats>;
   static constexpr auto s_collect_samples     = boost::sml::state<class StateCollectSamples>;
   static constexpr auto s_sample_wait         = boost::sml::state<class StateSampleWait>;
   static constexpr auto s_verify_sample       = boost::sml::state<class StateVerifySample>;
   static constexpr auto s_check_samples_count = boost::sml::state<class StateCheckSamplesCount>;
   static constexpr auto s_evaluate_stats      = boost::sml::state<class StateEvaluateStats>;
   static constexpr auto s_disable_self_test   = boost::sml::state<class StateDisableSelfTest>;

   auto operator()() const
   {
      using namespace boost::sml;

      // actions
      constexpr auto reset_timer = [](StateHandler& state)
      { state.reset_timer(); };

      constexpr auto reset_stats = [](StateHandler& state)
      { state.reset_stats(); };

      constexpr auto tick_timer = [](StateHandler& state)
      { state.tick_timer(); };

      constexpr auto read_data = [](StateHandler& state)
      { state.read_data(); };

      constexpr auto convert_raw_data = [](StateHandler& state)
      { state.convert_raw_data(); };

      constexpr auto update_stats = [](StateHandler& state)
      { state.update_stats(); };

      constexpr auto calculate_stats_and_bias = [](StateHandler& state)
      { state.calculate_stats_and_bias(); };

      constexpr auto mark_self_test_fail = [](StateHandler& state)
      { state.mark_self_test_fail(); };

      constexpr auto mark_self_test_pass = [](StateHandler& state)
      { state.mark_self_test_pass(); };

      constexpr auto set_bus_error = [](StateHandler& state)
      { state.set_error(imu_sensor::ImuSensorError::bus_error); };

      constexpr auto set_data_pattern_error = [](StateHandler& state)
      { state.set_error(imu_sensor::ImuSensorError::data_pattern_error); };

      constexpr auto set_out_of_range_data_error = [](StateHandler& state)
      { state.set_error(imu_sensor::ImuSensorError::out_of_range_data_error); };

      constexpr auto set_unstable_samples_error = [](StateHandler& state)
      { state.set_unstable_samples_error(); };

      // guards
      constexpr auto is_data_pattern_ok = [](const StateHandler& state)
      { return state.is_data_pattern_ok(); };

      constexpr auto is_data_valid = [](const StateHandler& state)
      { return state.is_data_valid(); };

      constexpr auto receive_wait_timeout = [](const StateHandler& state)
      { return state.receive_wait_timeout(); };

      constexpr auto all_samples_collected = [](const StateHandler& state)
      { return state.all_samples_collected(); };

      constexpr auto is_sensor_sane = [](const StateHandler& state)
      { return state.is_sensor_sane(); };

      // events
      static constexpr auto e_tick         = event<EventTick>;
      static constexpr auto e_receive_done = event<EventReceiveDone>;

      // clang-format off
      return make_transition_table(
          // From State     | Event           | Guard                   | Action                                             | To State
          *s_reset_stats                                                / reset_stats                                        = s_collect_samples,

          s_collect_samples + e_tick                                    / (reset_timer, read_data)                           = s_sample_wait,

          s_sample_wait     + e_receive_done  [is_data_pattern_ok]      / convert_raw_data                                   = s_verify_sample,
          s_sample_wait     + e_receive_done  [!is_data_pattern_ok]     / (set_data_pattern_error, mark_self_test_fail)      = X,
          s_sample_wait     + e_tick          [!receive_wait_timeout]   / tick_timer,
          s_sample_wait     + e_tick          [receive_wait_timeout]    / (set_bus_error, mark_self_test_fail)               = X,

          s_verify_sample                     [is_data_valid]           / update_stats                                       = s_check_samples_count,
          s_verify_sample                     [!is_data_valid]          / (set_out_of_range_data_error, mark_self_test_fail) = X,

          s_check_samples_count               [!all_samples_collected]                                                       = s_collect_samples,
          s_check_samples_count               [all_samples_collected]   / calculate_stats_and_bias                           = s_evaluate_stats,

          s_evaluate_stats                    [!is_sensor_sane]         / (set_unstable_samples_error, mark_self_test_fail)  = X,
          s_evaluate_stats                    [is_sensor_sane]          / mark_self_test_pass                                = X

      );
      // clang-format on
   }
};

}   // namespace mpu6500

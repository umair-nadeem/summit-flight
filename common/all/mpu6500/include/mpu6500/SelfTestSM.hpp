#pragma once

namespace mpu6500
{

template <typename StateHandler>
struct SelfTestStateMachine
{
   // states
   static constexpr auto enable_self_test      = boost::sml::state<class StateEnableSelfTest>;
   static constexpr auto read_self_test_values = boost::sml::state<class StateReadSelfTestValues>;
   static constexpr auto evaluate_self_test    = boost::sml::state<class StateEvaluateSelfTest>;
   static constexpr auto disable_self_test     = boost::sml::state<class StateDisableSelfTest>;

   // auto operator()() const
   // {
   //    using namespace boost::sml;

   //    // actions

   //    // clang-format off
   //    // return make_transition_table(
   //    //     // From State       | Event       | Guard                    | Action                           | To State
   //    //     *s_power_reset      + e_tick                                 / (reset_timer, power_reset)       = s_power_reset_wait,
   //    //     s_power_reset_wait  + e_tick      [!power_reset_wait_over]   / tick_timer

   //    // );
   //    // clang-format on
   // }
};

}   // namespace mpu6500

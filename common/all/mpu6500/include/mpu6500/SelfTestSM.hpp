#pragma once

namespace mpu6500
{

struct SelfTestStateMachine
{
   // states
   static constexpr auto enable_self_test      = boost::sml::state<class StateEnableSelfTest>;
   static constexpr auto read_self_test_values = boost::sml::state<class StateReadSelfTestValues>;
   static constexpr auto evaluate_self_test    = boost::sml::state<class StateEvaluateSelfTest>;
   static constexpr auto disable_self_test     = boost::sml::state<class StateDisableSelfTest>;
};

}   // namespace mpu6500

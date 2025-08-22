#pragma once

namespace mpu6500
{

struct ConfigStateMachine
{
   // states
   static constexpr auto set_clock       = boost::sml::state<class StateSetClock>;
   static constexpr auto gyro_config     = boost::sml::state<class StateGyroConfig>;
   static constexpr auto accel_config    = boost::sml::state<class StateAccelConfig>;
   static constexpr auto config          = boost::sml::state<class StateConfig>;
   static constexpr auto verify_config   = boost::sml::state<class StateVerify_Config>;
   static constexpr auto set_sample_rate = boost::sml::state<class StateSetSampleRate>;
};

}   // namespace mpu6500

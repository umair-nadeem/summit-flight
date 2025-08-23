#pragma once

#include <boost/sml.hpp>

#include "Events.hpp"

namespace mpu6500
{

template <typename StateHandler>
struct SelfTestStateMachine
{
   // states
   static constexpr auto s_dlpf_config        = boost::sml::state<class StateDlpfConfig>;
   static constexpr auto s_sample_rate        = boost::sml::state<class StateSampleRate>;
   static constexpr auto s_evaluate_self_test = boost::sml::state<class StateEvaluateSelfTest>;
   static constexpr auto s_disable_self_test  = boost::sml::state<class StateDisableSelfTest>;

   auto operator()() const
   {
      using namespace boost::sml;

      // actions
      constexpr auto mark_self_test_pass = [](StateHandler& state)
      { state.mark_self_test_pass(); };

      // events
      static constexpr auto e_tick = event<EventTick>;

      // clang-format off
      return make_transition_table(
         // From State          | Event       | Guard                    | Action                           | To State
         *s_dlpf_config         + e_tick                                                                 = s_sample_rate,
         s_sample_rate          + e_tick                                 / mark_self_test_pass               = X
         
      );
      // clang-format on
   }
};
// SelfTest (cold boot only; never in flight unless you explicitly command it)

// Goal: one-time built-in self test and OTP comparison.

// Factory ST codes (OTP): read SELF_TEST_X_GYRO…Z_GYRO (0x00–0x02) and SELF_TEST_X/Y/Z_ACCEL (0x0D–0x0F).
// TDK InvenSense

// Procedure (per InvenSense guidance):

// Configure normal ranges (see next state), read a few averages (“baseline”).

// Enable self-test:

// GYRO_CONFIG (27): set XG_ST=1, YG_ST=1, ZG_ST=1 (bits7:5=111).

// ACCEL_CONFIG (28): set XA_ST=1, YA_ST=1, ZA_ST=1 (bits7:5=111).

// Wait ≥200 ms, acquire averages (“ST”).
// TDK InvenSense

// Evaluate: if ST_OTP≠0, check ratios vs OTP; if ST_OTP==0, check absolute limits. Typical pass criteria used in production code (reflecting TDK app note) are:

// Gyro: (GXST/GXST_OTP) > 0.5 for each axis, or |GXST| ≥ 60 dps if OTP=0.

// Accel: 0.5 < (AXST/AXST_OTP) < 1.5 for each axis, or 225 mg ≤ |AXST| ≤ 675 mg if OTP=0.
// If any axis fails → flag degraded sensor and don’t arm flight; you may still continue to config to allow maintenance mode.
// Microsoft Learn
// TDK InvenSense

// Disable self-test bits (clear bits7:5 in both registers).

}   // namespace mpu6500

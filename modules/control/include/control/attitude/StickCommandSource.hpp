#pragma once

#include "boundaries/SharedData.hpp"
#include "control/attitude/StickCommand.hpp"

namespace control::attitude
{

template <typename StickCommandFilter,
          typename ThrottleCurve>
class StickCommandSource
{
   using StickCommandSubscriber = boundaries::SharedData<StickCommand>;

public:
   explicit StickCommandSource(StickCommandFilter&           roll_input_lpf,
                               StickCommandFilter&           pitch_input_lpf,
                               StickCommandFilter&           yaw_input_lpf,
                               ThrottleCurve&                throttle_curve,
                               const StickCommandSubscriber& stick_command_subscriber,
                               const float                   throttle_min,
                               const float                   throttle_max)
       : m_roll_input_lpf{roll_input_lpf},
         m_pitch_input_lpf{pitch_input_lpf},
         m_yaw_input_lpf{yaw_input_lpf},
         m_throttle_curve{throttle_curve},
         m_stick_command_subscriber{stick_command_subscriber},
         m_throttle_min{throttle_min},
         m_throttle_max{throttle_max}
   {
   }

   StickCommand get_raw() const
   {
      auto stick_command = m_stick_command_subscriber.get_latest();
      process_stick_command(stick_command.data);
      return stick_command.data;
   }

   StickCommand apply_filter(const StickCommand& command, const float dt_s)
   {
      return StickCommand{
          .throttle = command.throttle,
          .roll     = m_roll_input_lpf.apply(command.roll, dt_s),
          .pitch    = m_pitch_input_lpf.apply(command.pitch, dt_s),
          .yaw      = m_yaw_input_lpf.apply(command.yaw, dt_s)};
   }

   void reset()
   {
      m_roll_input_lpf.reset();
      m_pitch_input_lpf.reset();
      m_yaw_input_lpf.reset();
   }

private:
   void process_stick_command(StickCommand& command) const
   {
      // Pilot Command Mapping: perform pitch sign inversion (flight stick pullback -> nose up)
      command.pitch = -command.pitch;

      command.roll     = std::clamp(command.roll, -1.0f, 1.0f);
      command.pitch    = std::clamp(command.pitch, -1.0f, 1.0f);
      command.yaw      = std::clamp(command.yaw, -1.0f, 1.0f);
      command.throttle = std::clamp(command.throttle, 0.0f, 1.0f);

      command.throttle = m_throttle_curve.apply(command.throttle);
      command.throttle = std::clamp(command.throttle, m_throttle_min, m_throttle_max);
   }

   StickCommandFilter&           m_roll_input_lpf;
   StickCommandFilter&           m_pitch_input_lpf;
   StickCommandFilter&           m_yaw_input_lpf;
   ThrottleCurve&                m_throttle_curve;
   const StickCommandSubscriber& m_stick_command_subscriber;
   const float                   m_throttle_min;
   const float                   m_throttle_max;
};

}   // namespace control::attitude

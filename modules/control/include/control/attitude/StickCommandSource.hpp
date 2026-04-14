#pragma once

#include "boundaries/SharedData.hpp"
#include "control/attitude/StickCommand.hpp"

namespace control::attitude
{

template <typename ThrottleCurve>
class StickCommandSource
{
   using StickCommandSubscriber = boundaries::SharedData<StickCommand>;

public:
   explicit StickCommandSource(ThrottleCurve&                throttle_curve,
                               const StickCommandSubscriber& stick_command_subscriber,
                               const float                   throttle_min,
                               const float                   throttle_max)
       : m_throttle_curve{throttle_curve},
         m_stick_command_subscriber{stick_command_subscriber},
         m_throttle_min{throttle_min},
         m_throttle_max{throttle_max}
   {
   }

   StickCommand get() const
   {
      auto stick_command = m_stick_command_subscriber.get_latest();
      process_stick_command(stick_command.data);
      return stick_command.data;
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

   ThrottleCurve&                m_throttle_curve;
   const StickCommandSubscriber& m_stick_command_subscriber;
   const float                   m_throttle_min;
   const float                   m_throttle_max;
};

}   // namespace control::attitude

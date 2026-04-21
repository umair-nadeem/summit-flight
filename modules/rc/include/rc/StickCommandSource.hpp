#pragma once

#include "boundaries/SharedData.hpp"
#include "rc/StickCommand.hpp"
#include "rc/StickCommandSourceParams.hpp"

namespace rc
{

template <typename StickCommandFilter,
          typename ThrottleCurve>
class StickCommandSource
{
   using StickCommandSubscriber = boundaries::SharedData<StickCommand>;

public:
   explicit StickCommandSource(StickCommandFilter&             roll_input_lpf,
                               StickCommandFilter&             pitch_input_lpf,
                               StickCommandFilter&             yaw_input_lpf,
                               ThrottleCurve&                  throttle_curve,
                               const StickCommandSubscriber&   stick_command_subscriber,
                               const StickCommandSourceParams& params)
       : m_roll_input_lpf{roll_input_lpf},
         m_pitch_input_lpf{pitch_input_lpf},
         m_yaw_input_lpf{yaw_input_lpf},
         m_throttle_curve{throttle_curve},
         m_stick_command_subscriber{stick_command_subscriber},
         m_params{params}
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
          .roll     = m_roll_input_lpf.apply(command.roll, dt_s),
          .pitch    = m_pitch_input_lpf.apply(command.pitch, dt_s),
          .yaw      = m_yaw_input_lpf.apply(command.yaw, dt_s),
          .throttle = command.throttle};
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
      command.throttle = std::clamp(command.throttle, m_params.throttle_min, m_params.throttle_max);
   }

   StickCommandFilter&             m_roll_input_lpf;
   StickCommandFilter&             m_pitch_input_lpf;
   StickCommandFilter&             m_yaw_input_lpf;
   ThrottleCurve&                  m_throttle_curve;
   const StickCommandSubscriber&   m_stick_command_subscriber;
   const StickCommandSourceParams& m_params;
};

}   // namespace rc

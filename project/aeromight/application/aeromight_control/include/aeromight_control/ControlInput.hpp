#pragma once

#include "aeromight_boundaries/FlightControlSetpoints.hpp"
#include "boundaries/SharedData.hpp"

namespace aeromight_control
{

template <typename ThrottleCurve>
class ControlInput
{
   using FlightControlSetpointsSubscriber = boundaries::SharedData<aeromight_boundaries::FlightControlSetpoints>;

public:
   explicit ControlInput(ThrottleCurve&                          throttle_curve,
                         const FlightControlSetpointsSubscriber& flight_control_setpoints_subscriber,
                         const float                             throttle_min,
                         const float                             throttle_max)
       : m_throttle_curve{throttle_curve},
         m_flight_control_setpoints_subscriber{flight_control_setpoints_subscriber},
         m_throttle_min{throttle_min},
         m_throttle_max{throttle_max}
   {
   }

   aeromight_boundaries::FlightControlSetpoints get_flight_control_setpoints() const
   {
      auto flight_control_setpoints = m_flight_control_setpoints_subscriber.get_latest();
      apply_stick_input_transformation(flight_control_setpoints.data);
      return flight_control_setpoints.data;
   }

private:
   void apply_stick_input_transformation(aeromight_boundaries::FlightControlSetpoints& setpoints) const
   {
      // Pilot Command Mapping: perform pitch sign inversion (flight stick pullback -> nose up)
      setpoints.pitch    = -setpoints.pitch;
      setpoints.throttle = std::clamp(setpoints.throttle, 0.0f, 1.0f);
      setpoints.throttle = m_throttle_curve.apply(setpoints.throttle);
      setpoints.throttle = std::clamp(setpoints.throttle, m_throttle_min, m_throttle_max);
   }

   ThrottleCurve&                          m_throttle_curve;
   const FlightControlSetpointsSubscriber& m_flight_control_setpoints_subscriber;
   const float                             m_throttle_min;
   const float                             m_throttle_max;
};

}   // namespace aeromight_control

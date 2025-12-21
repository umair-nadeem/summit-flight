#pragma once

namespace aeromight_boundaries
{

struct ActuatorSetpoints
{
   struct Torque
   {
      float m1{};
      float m2{};
      float m3{};
      float m4{};
   };

   Torque torque{};
   bool   enabled{false};
};

}   // namespace aeromight_boundaries

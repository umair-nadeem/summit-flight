#pragma once

#include <cstdint>

namespace aeromight_control
{

struct StateEstimation
{
   uint32_t timestamp_ms{};

   // Euler angles
   float roll;
   float pitch;
   float yaw;

   // angular rates
   float p;
   float q;
   float r;

   // altitude control
   float altitude;
   float vertical_velocity;
};

}   // namespace aeromight_control

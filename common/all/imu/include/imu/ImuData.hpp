#pragma once

#include <optional>

#include "math/Vector3.hpp"

namespace imu
{

struct ImuData
{
   struct Status
   {
      bool calibration_ongoing{false};
   };

   std::optional<math::Vector3> accel_mps2;      // Acceleration [m/s²]
   std::optional<math::Vector3> gyro_radps;      // Angular velocity [rad/s]
   std::optional<float>         temperature_c;   // Temperature [°C]
   Status                       status{};
};

}   // namespace imu

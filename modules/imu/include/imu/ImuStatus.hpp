#pragma once

#include "ImuError.hpp"

namespace imu
{

struct ImuStatus
{
   ErrorBits error{0};
   bool      available{false};
   bool      calibration_done{false};
   bool      calibration_failure{false};
};

}   // namespace imu

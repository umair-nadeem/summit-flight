#pragma once

#include <bitset>
#include <cstdint>

namespace imu
{

enum class ImuError : uint8_t
{
   non_stationary_calibration_error,   // platform is not stationary during calibration phase
   unstable_gyro_error,                // non-consistent gyro samples collected in self-test
   unstable_accel_error,               // non-consistent accel samples collected in self-test
   max_error
};

using ErrorBits = std::bitset<static_cast<uint8_t>(ImuError::max_error)>;

}   // namespace imu

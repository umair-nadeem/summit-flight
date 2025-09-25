#pragma once

#include <bitset>
#include <cstdint>

namespace imu_sensor
{

enum class ImuSensorError : uint8_t
{
   bus_error,                 // bus communication failure
   id_mismatch_error,         // id mismatch
   config_mismatch_error,     // config mismatch
   data_pattern_error,        // all zeros/all ones data etc.
   out_of_range_data_error,   // non-zero but implausible data
   unstable_gyro_error,       // non-consistent gyro samples collected in self-test
   unstable_accel_error,      // non-consistent accel samples collected in self-test
   max_error
};

using ErrorBits = std::bitset<static_cast<uint8_t>(ImuSensorError::max_error)>;

}   // namespace imu_sensor

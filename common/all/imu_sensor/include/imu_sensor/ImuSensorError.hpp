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
   max_error
};

using ErrorBits = std::bitset<static_cast<uint8_t>(ImuSensorError::max_error)>;

}   // namespace imu_sensor

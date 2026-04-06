#pragma once

#include <bitset>
#include <cstdint>

#include "types/types.hpp"

namespace imu_sensor
{

enum class ImuSensorError : types::ErrorBitsType
{
   bus_error,                 // bus communication failure
   id_mismatch_error,         // id mismatch
   config_mismatch_error,     // config mismatch
   data_pattern_error,        // all zeros/all ones data etc.
   out_of_range_data_error,   // non-zero but implausible data
   max_error
};

using ErrorBits = std::bitset<static_cast<types::ErrorBitsType>(ImuSensorError::max_error)>;

}   // namespace imu_sensor

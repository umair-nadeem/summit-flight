#pragma once

#include <cstdint>

namespace barometer_sensor
{

enum class BarometerSensorError : uint8_t
{
   bus_error = 0,                // bus communication failure/timeout
   id_mismatch_error,            // id mismatch
   config_mismatch_error,        // config mismatch
   coefficients_pattern_error,   // all zeros/all ones calibration coefficients
   sensor_error,                 // sensor error register bits
   data_pattern_error,           // all zeros/all ones data etc.
   out_of_range_data_error,      // non-zero but implausible data
   max_error
};

}   // namespace barometer_sensor

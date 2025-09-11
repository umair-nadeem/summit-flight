#pragma once

#include <cstdint>

namespace barometer_sensor
{

enum class BarometerSensorError : uint8_t
{
   bus_error = 0,           // bus communication failure
   id_mismatch_error,       // id mismatch etc.
   config_mismatch_error,   // config mismatch
   sensor_error,            // all zeros etc.
   data_error,              // non-zero but implausible data
   max_error
};

}   // namespace barometer_sensor

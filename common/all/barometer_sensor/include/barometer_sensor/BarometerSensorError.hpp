#pragma once

namespace barometer_sensor
{

enum class BarometerSensorError : uint8_t
{
   bus_error    = 0,   // bus communication failure
   sensor_error = 1,   // all zeros etc.
   data_error   = 2,   // non-zero but implausible data
   max_error
};

}   // namespace barometer_sensor

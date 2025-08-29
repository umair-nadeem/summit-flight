#pragma once

namespace barometer_sensor
{

enum class BarometerSensorError
{
   none,
   bus_error,      // bus communication failure
   sensor_error,   // all zeros etc.
   data_error,     // non-zero but implausible data
};

}   // namespace barometer_sensor

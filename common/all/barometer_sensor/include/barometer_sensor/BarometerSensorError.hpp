#pragma once

namespace barometer_sensor
{

enum class BarometerSensorError : uint8_t
{
   bus_error         = 0,   // bus communication failure
   id_mismatch_error = 1,   // id mismatch etc.
   sensor_error      = 2,   // all zeros etc.
   data_error        = 3,   // non-zero but implausible data
   max_error
};

}   // namespace barometer_sensor

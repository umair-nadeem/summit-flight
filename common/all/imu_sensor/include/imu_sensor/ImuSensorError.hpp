#pragma once

namespace imu_sensor
{

enum class ImuSensorError : uint8_t
{
   bus_error    = 0,   // bus communication failure
   sensor_error = 1,   // all zeros etc.
   data_error   = 2,   // non-zero but implausible data
   max_error
};

}   // namespace imu_sensor

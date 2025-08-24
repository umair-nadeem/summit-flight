#pragma once

namespace imu_sensor
{

enum class ImuSensorError
{
   none,
   bus_error,      // bus communication failure
   sensor_error,   // all zeros etc.
   data_error,     // implausible but non-zero data
};

}   // namespace imu_sensor

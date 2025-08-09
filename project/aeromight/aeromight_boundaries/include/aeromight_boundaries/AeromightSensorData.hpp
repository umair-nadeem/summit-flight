#pragma once

#include "boundaries/SensorDataStorage.hpp"
#include "imu_sensor/ImuData.hpp"
#include "sys_time/ClockSource.hpp"

namespace aeromight_boundaries
{

struct AeromightSensorData
{
   boundaries::SensorDataStorage<imu_sensor::ImuData, sys_time::ClockSource> imu_sensor_data_storage{};
};

extern AeromightSensorData aeromight_sensor_data;

}   // namespace aeromight_boundaries

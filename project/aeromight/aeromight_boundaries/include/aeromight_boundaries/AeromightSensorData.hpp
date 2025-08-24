#pragma once

#include "boundaries/SharedData.hpp"
#include "imu_sensor/ImuData.hpp"
#include "imu_sensor/ImuHealth.hpp"

namespace aeromight_boundaries
{

struct AeromightSensorData
{
   boundaries::SharedData<imu_sensor::ImuData>   imu_sensor_data_storage{};
   boundaries::SharedData<imu_sensor::ImuHealth> imu_sensor_health_storage{};
};

extern AeromightSensorData aeromight_sensor_data;

}   // namespace aeromight_boundaries

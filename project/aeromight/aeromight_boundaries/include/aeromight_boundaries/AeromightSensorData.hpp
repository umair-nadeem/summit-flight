#pragma once

#include "EstimatorHealth.hpp"
#include "barometer_sensor/BarometerData.hpp"
#include "barometer_sensor/BarometerHealth.hpp"
#include "boundaries/SharedData.hpp"
#include "imu_sensor/ImuData.hpp"
#include "imu_sensor/ImuHealth.hpp"

namespace aeromight_boundaries
{

struct AeromightSensorData
{
   // IMU
   boundaries::SharedData<imu_sensor::ImuData>   imu_sensor_data_storage{};
   boundaries::SharedData<imu_sensor::ImuHealth> imu_sensor_health_storage{};

   // Barometer
   boundaries::SharedData<barometer_sensor::BarometerData>   barometer_sensor_data_storage{};
   boundaries::SharedData<barometer_sensor::BarometerHealth> barometer_sensor_health_storage{};

   // Estimator
   boundaries::SharedData<EstimatorHealth> estimator_health_storage{};
};

extern AeromightSensorData aeromight_sensor_data;

}   // namespace aeromight_boundaries

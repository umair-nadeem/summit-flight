#pragma once

#include "ActuatorSetpoint.hpp"
#include "ControlHealth.hpp"
#include "ControlSetpoints.hpp"
#include "EstimatorHealth.hpp"
#include "FlightManagerData.hpp"
#include "barometer_sensor/BarometerData.hpp"
#include "barometer_sensor/BarometerHealth.hpp"
#include "boundaries/SharedData.hpp"
#include "imu_sensor/ImuData.hpp"
#include "imu_sensor/ImuHealth.hpp"

namespace aeromight_boundaries
{

struct AeromightData
{
   // IMU
   boundaries::SharedData<imu_sensor::ImuData>   imu_sensor_data_storage{};
   boundaries::SharedData<imu_sensor::ImuHealth> imu_sensor_health_storage{};

   // Barometer
   boundaries::SharedData<barometer_sensor::BarometerData>   barometer_sensor_data_storage{};
   boundaries::SharedData<barometer_sensor::BarometerHealth> barometer_sensor_health_storage{};

   // Estimator
   boundaries::SharedData<EstimatorHealth> estimator_health_storage{};

   // Control
   boundaries::SharedData<ActuatorControl>  actuator_control{};
   boundaries::SharedData<ControlHealth>    control_health_storage{};
   boundaries::SharedData<ControlSetpoints> control_setpoints{};

   // Flight Manager
   boundaries::SharedData<FlightSetpoints> flight_setpoints{};
   boundaries::SharedData<RadioLinkStats>  radio_link_actuals{};
};

extern AeromightData aeromight_data;

}   // namespace aeromight_boundaries

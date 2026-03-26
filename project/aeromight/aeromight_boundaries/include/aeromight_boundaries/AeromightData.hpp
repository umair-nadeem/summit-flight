#pragma once

#include "ActuatorSetpoints.hpp"
#include "ControlHealth.hpp"
#include "EstimatorHealth.hpp"
#include "FlightControlSetpoints.hpp"
#include "RadioLinkActuals.hpp"
#include "SystemControlSetpoints.hpp"
#include "SystemStateInfo.hpp"
#include "barometer_sensor/BarometerData.hpp"
#include "barometer_sensor/BarometerHealth.hpp"
#include "boundaries/SharedData.hpp"
#include "imu/ImuData.hpp"
#include "imu/ImuStatus.hpp"

namespace aeromight_boundaries
{

struct AeromightData
{
   // IMU
   boundaries::SharedData<imu::ImuData>   imu_data_storage{};
   boundaries::SharedData<imu::ImuStatus> imu_health_storage{};

   // Barometer
   boundaries::SharedData<barometer_sensor::BarometerData>   barometer_data_storage{};
   boundaries::SharedData<barometer_sensor::BarometerHealth> barometer_health_storage{};

   // Estimator
   boundaries::SharedData<EstimatorHealth> estimator_health_storage{};

   // Control
   boundaries::SharedData<ActuatorControl>        actuator_control{};
   boundaries::SharedData<ControlHealth>          control_health_storage{};
   boundaries::SharedData<FlightControlSetpoints> flight_control_setpoints{};

   // System Manager
   boundaries::SharedData<SystemStateInfo>        system_state_info{};
   boundaries::SharedData<SystemControlSetpoints> system_control_setpoints{};

   // Radio Link
   boundaries::SharedData<RadioLinkActuals> radio_link_actuals{};
};

extern AeromightData aeromight_data;

}   // namespace aeromight_boundaries

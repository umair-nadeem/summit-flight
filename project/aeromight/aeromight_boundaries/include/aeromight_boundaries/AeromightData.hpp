#pragma once

#include "ActuatorSetpoints.hpp"
#include "ControlHealth.hpp"
#include "EstimatorHealth.hpp"
#include "FlightControlSetpoints.hpp"
#include "RadioLinkActuals.hpp"
#include "SystemControlSetpoints.hpp"
#include "SystemStateInfo.hpp"
#include "barometer/BarometerData.hpp"
#include "barometer/BarometerStatus.hpp"
#include "boundaries/SharedData.hpp"
#include "imu/ImuData.hpp"
#include "imu/ImuStatus.hpp"

namespace aeromight_boundaries
{

struct AeromightData
{
   // IMU
   boundaries::SharedData<imu::ImuData>   imu_data{};
   boundaries::SharedData<imu::ImuStatus> imu_health{};

   // Barometer
   boundaries::SharedData<barometer::BarometerData>   barometer_data{};
   boundaries::SharedData<barometer::BarometerStatus> barometer_health{};

   // Estimator
   boundaries::SharedData<EstimatorHealth> estimator_health{};

   // Control
   boundaries::SharedData<ActuatorControl>        actuator_control{};
   boundaries::SharedData<ControlHealth>          control_health{};
   boundaries::SharedData<FlightControlSetpoints> flight_control_setpoints{};

   // System Manager
   boundaries::SharedData<SystemStateInfo>        system_state_info{};
   boundaries::SharedData<SystemControlSetpoints> system_control_setpoints{};

   // Radio Link
   boundaries::SharedData<RadioLinkActuals> radio_link_actuals{};
};

extern AeromightData aeromight_data;

}   // namespace aeromight_boundaries

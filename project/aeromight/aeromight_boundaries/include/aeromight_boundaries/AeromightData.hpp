#pragma once

#include "ControlStatus.hpp"
#include "EstimatorStatus.hpp"
#include "SystemControlSetpoints.hpp"
#include "SystemState.hpp"
#include "barometer/BarometerData.hpp"
#include "barometer/BarometerStatus.hpp"
#include "boundaries/SharedData.hpp"
#include "control/attitude/StickCommand.hpp"
#include "imu/ImuData.hpp"
#include "imu/ImuStatus.hpp"
#include "power/battery/BatteryStatus.hpp"
#include "rc/crsf/LinkStats.hpp"

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
   boundaries::SharedData<EstimatorStatus> estimator_health{};

   // Control
   boundaries::SharedData<ControlStatus>                   control_health{};
   boundaries::SharedData<control::attitude::StickCommand> stick_command{};

   boundaries::SharedData<power::battery::BatteryStatus> battery_status{};

   // System Manager
   boundaries::SharedData<SystemState>            system_state_info{};
   boundaries::SharedData<SystemControlSetpoints> system_control_setpoints{};

   // Radio Link
   boundaries::SharedData<rc::crsf::LinkStats> link_stats_actuals{};
};

extern AeromightData aeromight_data;

}   // namespace aeromight_boundaries

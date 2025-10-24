#pragma once

namespace aeromight_health
{

enum class HealthMonitoringState
{
   startup,                                 // phase 1: grace period to allow peripherals/modules to be started
   wait_for_sensors_readiness,              // phase 2: wait for sensors to be in  operational state
   wait_for_estimation_control_readiness,   // phase 3: wait for estimation & control to be in operational state
   general_monitoring                       // phase 4: perform routine health checks and publish regular summaries
};

}   // namespace aeromight_health

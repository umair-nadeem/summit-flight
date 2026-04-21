#pragma once

namespace aeromight_health
{

struct HealthMonitoringParams
{
   uint32_t execution_period_ms                = 40u;
   uint32_t max_age_stale_imu_sensor_health_ms = execution_period_ms * 5u;
   uint32_t max_age_barometer_sensor_health_ms = execution_period_ms * 10u;
   uint32_t max_age_estimation_health_ms       = execution_period_ms * 15u;
   uint32_t max_age_control_health_ms          = execution_period_ms * 5u;
   bool     evaluate_barometer_health          = false;
};

}   // namespace aeromight_health

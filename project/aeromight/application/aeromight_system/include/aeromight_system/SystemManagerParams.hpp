#pragma once

#include <cstdint>

namespace aeromight_system
{

struct SystemManagerParams
{
   uint32_t execution_period_ms            = 20u;
   float    stick_input_deadband_abs       = 0.1f;
   uint8_t  good_uplink_quality_pct        = 50u;
   float    min_good_signal_rssi_dbm       = -110.0f;
   uint32_t max_age_stale_data_ms          = 1000u;
   uint32_t min_state_debounce_duration_ms = 100u;
   uint32_t timeout_sensors_readiness_ms   = 10'000u;
   uint32_t timeout_control_readiness_ms   = 2000u;
};

}   // namespace aeromight_system
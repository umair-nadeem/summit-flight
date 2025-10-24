#pragma once

#include <cstdint>

#include "SubsystemHealth.hpp"

namespace aeromight_boundaries
{

struct HealthSummary
{
   uint32_t timestamp_ms{};

   bool all_sensors_ready{};
   bool estimation_ready{};
   bool control_ready{};
   bool flight_critical_fault{};
   bool health_update_queue_failure{};

   SubsystemHealth imu_health{SubsystemHealth::init};
   SubsystemHealth barometer_health{SubsystemHealth::init};
   SubsystemHealth estimation_health{SubsystemHealth::init};
   SubsystemHealth control_health{SubsystemHealth::init};
};

static constexpr std::size_t health_summary_queue_len = 8u;

}   // namespace aeromight_boundaries

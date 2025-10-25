#pragma once

#include <cstdint>

#include "SubsystemHealth.hpp"

namespace aeromight_boundaries
{

struct HealthSummary
{
   uint32_t timestamp_ms{};

   bool        all_sensors_ready{};
   bool        estimation_ready{};
   bool        control_ready{};
   bool        flight_critical_fault{};
   std::size_t queue_failure_count{};

   SubsystemHealth imu_health{SubsystemHealth::init};
   SubsystemHealth barometer_health{SubsystemHealth::init};
   SubsystemHealth estimation_health{SubsystemHealth::init};
   SubsystemHealth control_health{SubsystemHealth::init};

   bool operator==(const HealthSummary& rhs) const noexcept
   {
      return ((timestamp_ms == rhs.timestamp_ms) &&
              (all_sensors_ready == rhs.all_sensors_ready) &&
              (estimation_ready == rhs.estimation_ready) &&
              (control_ready == rhs.control_ready) &&
              (flight_critical_fault == rhs.flight_critical_fault) &&
              (queue_failure_count == rhs.queue_failure_count) &&
              (imu_health == rhs.imu_health) &&
              (barometer_health == rhs.barometer_health) &&
              (all_sensors_ready == rhs.all_sensors_ready) &&
              (estimation_health == rhs.estimation_health) &&
              (control_health == rhs.control_health));
   }

   bool operator!=(const HealthSummary& rhs) const noexcept
   {
      return !(*this == rhs);
   }
};

static constexpr std::size_t health_summary_queue_len = 8u;

}   // namespace aeromight_boundaries

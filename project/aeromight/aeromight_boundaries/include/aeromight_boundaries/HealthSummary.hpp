#pragma once

#include <cstdint>

#include "SubsystemHealth.hpp"

namespace aeromight_boundaries
{

struct HealthSummary
{
   uint32_t timestamp_ms{};

   bool        imu_operational{};
   bool        imu_calibration_finished{};
   bool        barometer_operational{};
   bool        estimation_operational{};
   bool        control_operational{};
   std::size_t total_queue_failure_count{};

   SubsystemHealth imu_health{SubsystemHealth::init};
   SubsystemHealth barometer_health{SubsystemHealth::init};
   SubsystemHealth estimation_health{SubsystemHealth::init};
   SubsystemHealth control_health{SubsystemHealth::init};

   bool operator==(const HealthSummary& rhs) const noexcept
   {
      return ((timestamp_ms == rhs.timestamp_ms) &&
              (imu_operational == rhs.imu_operational) &&
              (imu_calibration_finished == rhs.imu_calibration_finished) &&
              (barometer_operational == rhs.barometer_operational) &&
              (estimation_operational == rhs.estimation_operational) &&
              (control_operational == rhs.control_operational) &&
              (total_queue_failure_count == rhs.total_queue_failure_count) &&
              (imu_health == rhs.imu_health) &&
              (barometer_health == rhs.barometer_health) &&
              (estimation_health == rhs.estimation_health) &&
              (control_health == rhs.control_health));
   }

   bool operator!=(const HealthSummary& rhs) const noexcept
   {
      return !(*this == rhs);
   }
};

static constexpr std::size_t health_summary_queue_depth = 8u;

}   // namespace aeromight_boundaries

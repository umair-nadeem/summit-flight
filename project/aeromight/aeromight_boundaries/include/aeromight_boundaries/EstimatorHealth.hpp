#pragma once

#include <bitset>
#include <cstdint>

#include "EstimatorState.hpp"

namespace aeromight_boundaries
{

struct EstimatorHealth
{
   enum class Status : uint8_t
   {
      valid_reference_pressure_acquired = 0,
      reference_pressure_estimate_timeout,
      reference_pressure_implausible,
      stale_imu_sensor_data,
      stale_baro_sensor_data,
      missing_valid_imu_data,
      missing_valid_baro_data,
      max,
   };

   using StatusBits = std::bitset<static_cast<uint8_t>(Status::max)>;

   EstimatorState state{EstimatorState::idle};
   StatusBits     status{};
   uint8_t        recovery_attempts{};
};

}   // namespace aeromight_boundaries

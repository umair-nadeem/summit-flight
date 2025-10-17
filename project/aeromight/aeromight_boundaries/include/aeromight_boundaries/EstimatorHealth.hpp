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
      valid_pressure_reference_acquired = 0,
      altitude_conversion_failed,
      pressure_reference_estimate_timeout,
      stale_sensor_data,
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

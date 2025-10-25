#pragma once

#include <bitset>
#include <cstdint>

#include "EstimatorState.hpp"

namespace aeromight_boundaries
{

struct EstimatorHealth
{
   enum class Error : uint8_t
   {
      reference_pressure_estimate_timeout = 0,
      reference_pressure_implausible,
      stale_imu_sensor_data,
      stale_baro_sensor_data,
      missing_valid_imu_data,
      missing_valid_baro_data,
      max,
   };

   using ErrorBits = std::bitset<static_cast<uint8_t>(Error::max)>;

   ErrorBits      error{0};
   EstimatorState state{EstimatorState::idle};
   uint8_t        recovery_attempts{0};
   bool           valid_reference_pressure_acquired{false};
};

}   // namespace aeromight_boundaries

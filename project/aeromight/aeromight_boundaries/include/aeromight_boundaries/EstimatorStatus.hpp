#pragma once

#include <bitset>
#include <cstdint>

#include "types/types.hpp"

namespace aeromight_boundaries
{

struct EstimatorStatus
{
   enum class Error : types::ErrorBitsType
   {
      stale_imu_sensor_data = 0,
      stale_baro_sensor_data,
      missing_valid_imu_data,
      missing_valid_baro_data,
      max_error,
   };

   using ErrorBits = std::bitset<static_cast<types::ErrorBitsType>(Error::max_error)>;

   bool      enabled{false};
   ErrorBits error{0};
};

}   // namespace aeromight_boundaries

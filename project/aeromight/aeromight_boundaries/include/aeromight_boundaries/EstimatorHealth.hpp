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
      fault_occurred,
      max,
   };

   using StatusBits = std::bitset<static_cast<uint8_t>(Status::max)>;

   EstimatorState state{EstimatorState::idle};
   StatusBits     status{};
   uint8_t        recovery_attempts{};
};

}   // namespace aeromight_boundaries

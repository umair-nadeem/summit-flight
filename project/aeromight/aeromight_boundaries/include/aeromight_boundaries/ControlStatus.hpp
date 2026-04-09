#pragma once

#include <bitset>
#include <cstdint>

#include "types/types.hpp"

namespace aeromight_boundaries
{

struct ControlStatus
{
   enum class Error : types::ErrorBitsType
   {
      invalid_estimation_sample,
      timing_jitter,
      max_error
   };

   using ErrorBits = std::bitset<static_cast<types::ErrorBitsType>(Error::max_error)>;

   bool      enabled{false};
   ErrorBits error{0};
};

}   // namespace aeromight_boundaries

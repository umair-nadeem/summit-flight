#pragma once

#include <bitset>
#include <cstdint>

#include "types/types.hpp"

namespace aeromight_boundaries
{

struct ControlHealth
{
   enum class Error : types::ErrorBitsType
   {
      invalid_estimation_data,
      max_error
   };

   using ErrorBits = std::bitset<static_cast<types::ErrorBitsType>(Error::max_error)>;

   bool      enabled{false};
   ErrorBits error{0};
};

}   // namespace aeromight_boundaries

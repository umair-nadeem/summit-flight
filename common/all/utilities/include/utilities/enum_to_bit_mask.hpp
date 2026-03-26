#pragma once

#include <cstdint>
#include <type_traits>

namespace utilities
{

static constexpr uint32_t maximum_bits = 32;

template <auto Enum>
constexpr uint32_t enum_to_bit_mask()
{
   using Type           = decltype(Enum);
   using UnderlyingType = std::underlying_type_t<Type>;

   static_assert(std::is_unsigned_v<UnderlyingType>);
   static_assert(static_cast<uint32_t>(Enum) < maximum_bits, "Event bits must not exceed maximum of 32");

   return (1u << static_cast<uint32_t>(Enum));
}

}   // namespace utilities

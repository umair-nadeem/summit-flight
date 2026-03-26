#pragma once

#include "types/types.hpp"

namespace event_handling
{

template <typename Event>
constexpr types::EventBitsType event_to_bit_mask(const Event e)
{
   return (1u << static_cast<types::EventBitsType>(e));
}

template <typename Event>
constexpr bool has_event(const types::EventBitsType bits, const Event e)
{
   return ((bits & event_to_bit_mask(e)) != 0u);
}

constexpr bool has_event(const types::EventBitsType bits, const types::EventBitsType event_mask)
{
   return ((bits & event_mask) != 0u);
}

}   // namespace event_handling

#pragma once

#include <type_traits>

#include "event_handling/event_check.hpp"
#include "types/types.hpp"

namespace event_handling
{

template <typename T, auto Handler>
class EventBinding
{
public:
   constexpr EventBinding(T& object, const types::EventBitsType event_mask)
       : m_object(object),
         m_event_mask(event_mask)
   {
   }

   void dispatch(const types::EventBitsType bits)
   {
      if (has_event(bits, m_event_mask))
      {
         (m_object.*Handler)();
      }
   }

private:
   T&                         m_object;
   const types::EventBitsType m_event_mask;
};

}   // namespace event_handling

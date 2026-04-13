#pragma once

#include <type_traits>

namespace math
{

template <typename Filter, typename... Args>
Filter make_filter(Args&&... args)
{
   if constexpr (std::is_constructible_v<Filter, Args...>)
   {
      return Filter{std::forward<Args>(args)...};
   }
   else
   {
      return Filter{};
   }
}

}   // namespace math

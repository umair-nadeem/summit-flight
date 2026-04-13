#pragma once

namespace math
{

struct NullFilter
{
   static float apply(const float x, [[maybe_unused]] const float dt_s = 0.0f) { return x; }

   static void reset() {}
};

}   // namespace math

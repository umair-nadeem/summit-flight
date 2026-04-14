#pragma once

#include <cstdint>

#include "math/Vector4.hpp"

namespace aeromight_boundaries
{

enum class ControlAxis : uint8_t
{
   roll   = 0,
   pitch  = 1u,
   yaw    = 2u,
   thrust = 3u
};

constexpr std::size_t idx(const ControlAxis axis)
{
   return static_cast<std::size_t>(axis);
}

static constexpr std::size_t num_axis = 3u;

}   // namespace aeromight_boundaries

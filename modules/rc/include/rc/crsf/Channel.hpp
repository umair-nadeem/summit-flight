#pragma once

#include "crsf/Crsf.hpp"
#include "error/error_handler.hpp"
#include "math/constants.hpp"

namespace rc::crsf
{

enum class Bistate
{
   low,
   high
};

enum class Tristate
{
   low,
   mid,
   high
};

enum class ChannelType : uint8_t
{
   unused = 0,
   raw_u16,
   normalized_float,   // [-1..1] or [0..1]
   bistate,
   tristate
};

struct ChannelConfig
{
   ChannelType type{ChannelType::unused};
   float       deadband{math::constants::epsilon};
   float       min_norm{-1.0f};
   float       max_norm{1.0f};
};

using ChannelValue = std::variant<std::monostate, uint16_t, float, Bistate, Tristate>;

using ChannelValues = std::array<ChannelValue, ::crsf::rc_channel_count>;

using ChannelConfigs = std::array<ChannelConfig, ::crsf::rc_channel_count>;

inline float get_float_channel(const ChannelValues& channels, const uint8_t idx)
{
   if (idx >= channels.size())
   {
      error::stop_operation();
   }

   return std::get<float>(channels[idx]);
}

inline Bistate get_bistate_channel(const ChannelValues& channels, const uint8_t idx)
{
   if (idx >= channels.size())
   {
      error::stop_operation();
   }

   return std::get<Bistate>(channels[idx]);
}

}   // namespace rc::crsf

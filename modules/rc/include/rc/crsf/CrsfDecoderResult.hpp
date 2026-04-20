#pragma once

namespace rc::crsf
{

enum class CrsfDecoderResult
{
   invalid_frame,
   rc_channels_update,
   link_stats_update,
   other_packet
};

}   // namespace rc::crsf

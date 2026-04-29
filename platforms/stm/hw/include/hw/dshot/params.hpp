#pragma once

#include <cstdint>

namespace hw::dshot
{

static constexpr uint16_t dshot_min = 48u;
static constexpr uint16_t dshot_max = 2047u;

static constexpr uint8_t frame_bits = 16u;
static constexpr uint8_t frame_gap  = 2u;
static constexpr uint8_t frame_len  = frame_bits + frame_gap;

}   // namespace hw::dshot

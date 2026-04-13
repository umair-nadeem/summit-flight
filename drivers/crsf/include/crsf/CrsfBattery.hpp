#pragma once

#include "params.hpp"

namespace crsf
{

#pragma pack(push, 1)
struct CrsfBattery
{
   int16_t voltage_10uv;        // Voltage (LSB = 10 uV)
   int16_t current_10ua;        // Current (LSB = 10 uA)
   int32_t capacity_used_mah;   // Capacity used (mAh) - Use only lower 24 bits!!
   uint8_t remaining_pct;       // Battery remaining (percent)
};
#pragma pack(pop)

}   // namespace crsf

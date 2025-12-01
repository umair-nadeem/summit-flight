#pragma once

#include "params.hpp"

namespace crsf
{

struct CrsfBattery
{
   int16_t voltage_v;           // Voltage (LSB = 10 µV)
   int16_t current_a;           // Current (LSB = 10 µA)
   int32_t capacity_used_mah;   // Capacity used (mAh) - Use only lower 24 bits!!
   uint8_t remaining_pct;       // Battery remaining (percent)
};

}   // namespace crsf

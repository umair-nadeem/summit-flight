#pragma once

namespace power::battery
{

struct BatteryStatus
{
   int32_t voltage_mv{};
   uint8_t remaining_pct{};
};

}   // namespace power::battery

#pragma once

namespace aeromight_battery
{

struct VoltageSenseConfig
{

   float    r1_ohm{};
   float    r2_ohm{};
   float    vref_v{};
   uint16_t resolution{};
};

}   // namespace aeromight_battery

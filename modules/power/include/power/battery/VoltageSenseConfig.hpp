#pragma once

namespace power::battery
{

struct VoltageSenseConfig
{

   float    r1_ohm{};
   float    r2_ohm{};
   float    vref_v{};
   uint16_t resolution{};
};

}   // namespace power::battery

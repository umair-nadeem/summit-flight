#pragma once

namespace power::battery
{

struct VoltageSenseConfig
{

   float    r1_ohm     = 100'000.0f;
   float    r2_ohm     = 22'000.0f;
   float    vref_v     = 3.3f;
   uint16_t resolution = 4095u;
};

}   // namespace power::battery

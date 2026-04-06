#pragma once

namespace bmp390
{

enum class SensorState
{
   stopped,
   setup,
   read_coefficients,
   operational,
   recovery,
   fault
};

}   // namespace bmp390

#pragma once

namespace mpu6500
{

enum class SensorState
{
   stopped,
   reset,
   validation,
   config,
   operational,
   soft_recovery,
   hard_recovery,
   fault
};

}   // namespace mpu6500

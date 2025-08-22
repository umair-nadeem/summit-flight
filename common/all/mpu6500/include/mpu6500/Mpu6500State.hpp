#pragma once

namespace mpu6500
{

enum class Mpu6500State
{
   stopped,
   reset,
   validation,
   self_test,
   config,
   operational,
   soft_recovery,
   hard_recovery,
   failure

};

}   // namespace mpu6500

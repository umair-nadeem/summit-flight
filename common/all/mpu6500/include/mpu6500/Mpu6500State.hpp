#pragma once

namespace mpu6500
{

enum class Mpu6500State
{
   stopped,
   init,
   config,
   operational,
   recovery,
   failure

};

}   // namespace mpu6500

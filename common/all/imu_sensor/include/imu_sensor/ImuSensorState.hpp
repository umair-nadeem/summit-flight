#pragma once

namespace imu_sensor
{

enum class ImuSensorState
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

}   // namespace imu_sensor

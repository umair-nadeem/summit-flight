#pragma once

namespace barometer_sensor
{

enum class BarometerSensorState
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

}   // namespace barometer_sensor

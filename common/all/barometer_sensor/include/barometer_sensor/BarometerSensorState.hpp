#pragma once

namespace barometer_sensor
{

enum class BarometerSensorState
{
   stopped,
   setup,
   self_test,
   operational,
   soft_recovery,
   hard_recovery,
   failure
};

}   // namespace barometer_sensor

#pragma once

namespace barometer_sensor
{

enum class BarometerSensorState
{
   stopped,
   setup,
   read_coefficients,
   operational,
   recovery,
   failure
};

}   // namespace barometer_sensor

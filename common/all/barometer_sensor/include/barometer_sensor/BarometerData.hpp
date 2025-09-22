#pragma once

#include <optional>

namespace barometer_sensor
{

struct BarometerData
{
   std::optional<float> pressure_pa;     // Pressure [Pa]
   std::optional<float> temperature_c;   // Temperature [°C]
};

}   // namespace  barometer_sensor

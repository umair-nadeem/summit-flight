#pragma once

#include <optional>

namespace barometer_sensor
{

struct BarometerData
{
   std::optional<float> pressure_pa;     // Pressure [Pa]
   std::optional<float> temperature_c;   // Temperature [°C]
   std::optional<float> altitude_m;      // Altitude [m]
};

}   // namespace  barometer_sensor

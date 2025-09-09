#pragma once

#include <optional>

namespace barometer_sensor
{

struct BarometerData
{
   float                pressure_pa;     // Pressure [Pa]
   float                altitude_m;      // Altitude [m]
   std::optional<float> temperature_c;   // Temperature [°C]
};

}   // namespace  barometer_sensor

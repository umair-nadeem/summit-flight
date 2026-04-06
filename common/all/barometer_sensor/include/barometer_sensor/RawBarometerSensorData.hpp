#pragma once

#include <optional>

#include "math/Vector3.hpp"

namespace barometer_sensor
{

struct RawBarometerSensorData
{
   std::size_t          count;
   float                pressure_pa;
   std::optional<float> temperature_c;
};

}   // namespace barometer_sensor

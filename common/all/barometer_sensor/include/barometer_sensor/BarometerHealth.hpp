#pragma once

#include "barometer_sensor/BarometerSensorError.hpp"

namespace barometer_sensor
{

struct BarometerHealth
{
   BarometerSensorError error{BarometerSensorError::none};
};

}   // namespace barometer_sensor

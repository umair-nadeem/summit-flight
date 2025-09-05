#pragma once

#include "barometer_sensor/BarometerSensorError.hpp"
#include "barometer_sensor/BarometerSensorState.hpp"

namespace barometer_sensor
{

struct BarometerHealth
{
   BarometerSensorError error{BarometerSensorError::none};
   BarometerSensorState state{BarometerSensorState::stopped};
};

}   // namespace barometer_sensor

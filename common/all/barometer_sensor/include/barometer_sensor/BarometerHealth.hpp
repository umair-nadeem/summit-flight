#pragma once

#include <bitset>

#include "barometer_sensor/BarometerSensorError.hpp"
#include "barometer_sensor/BarometerSensorState.hpp"

namespace barometer_sensor
{

struct BarometerHealth
{
   using ErrorBits = std::bitset<static_cast<uint8_t>(BarometerSensorError::max_error)>;

   ErrorBits            error{0};
   BarometerSensorState state{BarometerSensorState::stopped};
   std::size_t          read_failure_count{0};
   bool                 validation_ok{false};
   bool                 self_test_ok{false};
   bool                 config_ok{false};
};

}   // namespace barometer_sensor

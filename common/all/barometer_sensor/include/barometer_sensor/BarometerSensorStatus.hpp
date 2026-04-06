#pragma once

#include "BarometerSensorError.hpp"

namespace barometer_sensor
{

struct BarometerSensorStatus
{
   BarometerSensorErrorBits error{0};
   std::size_t              read_failure_count{0};
   bool                     fault{false};
   bool                     setup_ok{false};
   std::size_t              recovery_attempt_count{0};
};

}   // namespace barometer_sensor

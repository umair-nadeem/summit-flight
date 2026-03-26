#pragma once

#include "ImuSensorError.hpp"

namespace imu_sensor
{

struct ImuSensorStatus
{
   ErrorBits   error{0};
   std::size_t read_failure_count{0};
   bool        fault{false};
   bool        validation_ok{false};
   bool        config_ok{false};
};

}   // namespace imu_sensor

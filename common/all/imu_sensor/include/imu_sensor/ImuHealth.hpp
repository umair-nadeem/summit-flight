#pragma once

#include "ImuSensorError.hpp"
#include "ImuSensorState.hpp"

namespace imu_sensor
{

struct ImuHealth
{
   ErrorBits      error{0};
   ImuSensorState state{ImuSensorState::stopped};
   std::size_t    read_failure_count{0};
   bool           validation_ok{false};
   bool           config_ok{false};
   bool           self_test_ok{false};
};

}   // namespace imu_sensor

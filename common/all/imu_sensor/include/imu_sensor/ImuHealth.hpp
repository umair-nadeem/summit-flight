#pragma once

#include "ImuSensorError.hpp"
#include "ImuSensorState.hpp"

namespace imu_sensor
{

struct ImuHealth
{
   ImuSensorError error{imu_sensor::ImuSensorError::none};
   ImuSensorState state{imu_sensor::ImuSensorState::stopped};
   std::size_t    read_failure_count{0};
   bool           validation_ok{false};
   bool           self_test_ok{false};
   bool           config_ok{false};
};

}   // namespace imu_sensor

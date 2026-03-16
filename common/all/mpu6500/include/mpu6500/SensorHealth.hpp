#pragma once

#include "SensorError.hpp"
#include "SensorState.hpp"

namespace mpu6500
{

struct SensorHealth
{
   ErrorBits   error{0};
   SensorState state{SensorState::stopped};
   std::size_t read_failure_count{0};
   bool        validation_ok{false};
   bool        config_ok{false};
   bool        self_test_ok{false};
};

}   // namespace mpu6500

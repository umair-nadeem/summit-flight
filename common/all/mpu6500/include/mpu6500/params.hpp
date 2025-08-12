#pragma once

namespace mpu6500::params
{

static constexpr uint8_t num_bytes_command       = 1u;
static constexpr uint8_t num_bytes_accelerometer = 6u;
static constexpr uint8_t num_bytes_gyroscope     = 6u;
static constexpr uint8_t num_bytes_temperature   = 2u;
static constexpr uint8_t num_bytes_transaction   = num_bytes_command + num_bytes_accelerometer + num_bytes_gyroscope + num_bytes_temperature;

}   // namespace mpu6500::params

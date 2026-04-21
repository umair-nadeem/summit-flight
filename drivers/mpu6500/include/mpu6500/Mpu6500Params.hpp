#pragma once

namespace mpu6500
{

struct Mpu6500Params
{
   uint8_t  read_failures_limit                  = 5u;
   uint8_t  execution_period_ms                  = 4u;
   uint32_t receive_wait_timeout_ms              = 2u * execution_period_ms;
   uint8_t  sample_rate_divider                  = 0x03;
   uint8_t  dlpf_config                          = 0x03;
   uint8_t  gyro_full_scale                      = 0x03;
   uint8_t  accel_full_scale                     = 0x03;
   uint8_t  accel_a_dlpf_config                  = 0x03;
   float    gyro_range_plausibility_margin_radps = 6.0f;
   float    accel_range_plausibility_margin_mps2 = 20.0f;
};

}   // namespace mpu6500

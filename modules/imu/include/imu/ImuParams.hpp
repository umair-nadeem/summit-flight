#pragma once

namespace imu
{

struct ImuParams
{
   uint16_t num_calibration_samples = 400u;
   float    gyro_tolerance_radps    = 0.1f;
   float    accel_tolerance_mps2    = 1.5f;
   bool     front_left_up_frame     = true;
};

}   // namespace imu

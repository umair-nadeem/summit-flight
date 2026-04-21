#pragma once

namespace aeromight_estimation
{

struct AltitudeEkfParams
{
   float process_noise_z                = 1.0f;      // altitude noise
   float process_noise_vz               = 5.0f;      // vertical velocity noise
   float process_noise_accel_bias       = 0.0001f;   // accel bias noise
   float measurement_noise_baro         = 0.5f;      // R: measurement noise
   float tilt_gating_attitude_angle_rad = 0.2f;      // max roll, pitch angle before accel is tilt-compensated
   float tilt_gating_accel_weight       = 0.08f;     // accel_z weightage for tilt-compensation
};

}   // namespace aeromight_estimation

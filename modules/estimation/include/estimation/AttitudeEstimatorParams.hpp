#pragma once

namespace estimation
{

struct AttitudeEstimatorParams
{
   float accelerometer_weight = 0.2f;
   float gyro_bias_weight     = 0.1f;
};

}   // namespace estimation

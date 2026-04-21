#pragma once

namespace aeromight_estimation
{

struct EstimationParams
{
   bool     run_altitude_estimation         = false;
   uint32_t max_age_imu_data_ms             = 40u;
   uint32_t max_age_baro_data_ms            = 400u;
   float    max_valid_imu_sample_dt_s       = 0.02f;
   float    max_valid_barometer_sample_dt_s = 10.0f;
};

}   // namespace aeromight_estimation

#pragma once

#include "math/Vector3.hpp"

namespace aeromight_control
{

struct ControlParams
{
   uint32_t    max_age_state_estimation_data_ms = 40u;
   float       min_dt_s                         = 0.002f;
   float       max_dt_s                         = 0.010f;
   float       thrust_linearization_factor      = 0.8f;
   float       throttle_arming                  = 0.01f;
   float       throttle_gate_integrator         = 0.15f;
   bool        run_attitude_controller          = true;
   float       max_tilt_angle_rad               = 30 * math::constants::deg_to_rad;
   math::Vec3f max_rate{3.0f, 3.0f, 3.0f};
};

}   // namespace aeromight_control

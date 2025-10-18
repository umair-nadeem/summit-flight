#pragma once

#include "aeromight_control/EkfState.hpp"
#include "math/Matrix.hpp"
#include "math/Quaternion.hpp"

namespace aeromight_control
{

class AltitudeEkf
{
public:
   explicit AltitudeEkf(const float process_noise_z,
                        const float process_noise_vz,
                        const float process_noise_accel_bias,
                        const float measurement_noise_baro,
                        const float tilt_gating_attitude_angle_rad,
                        const float tilt_gating_accel_weight);

   void predict(const math::Vector3& accel_body_mps2, const math::Quaternion& attitude, const float dt_s);
   void update(const float baro_altitude_m);
   void reset();

   EkfState get_ekf_state() const;

private:
   void enfore_covariance_matrix_symmetry();

   static constexpr float       initial_uncertainty         = 10.0f;
   static constexpr float       min_recognizable_covariance = 1e-6f;
   static constexpr std::size_t num_state_dimensions        = 3u;

   const float                                                     m_process_noise_z;                  // altitude noise
   const float                                                     m_process_noise_vz;                 // vertical velocity noise
   const float                                                     m_process_noise_accel_bias;         // accel bias noise
   const float                                                     m_measurement_noise_baro;           // R: measurement noise
   const float                                                     m_tilt_gating_attitude_angle_rad;   // max roll, pitch angle before accel is tilt-compensated
   const float                                                     m_tilt_gating_accel_weight;         // accel_z weightage for tilt-compensation
   EkfState                                                        m_state{};                          // state vector
   math::Matrix<float, num_state_dimensions, num_state_dimensions> m_p{};                              // P: covariance matrix
   math::Matrix<float, num_state_dimensions, num_state_dimensions> m_q{};                              // Q: process noise
   math::Matrix<float, num_state_dimensions, num_state_dimensions> m_f_d{};                            // F_d: Discretized Jacobian of f(x_dot,u)
   math::Matrix<float, num_state_dimensions, num_state_dimensions> m_identity{};                       // F_d: Discretized Jacobian of f(x_dot,u)
   float                                                           m_last_accel_world_z{};
   bool                                                            m_is_initialized{false};
};

}   // namespace aeromight_control

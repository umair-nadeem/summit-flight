#include "aeromight_control/AltitudeEkf.hpp"

namespace
{

constexpr float transform_accel_to_world_z(const math::Vector3& accel_body, const math::Quaternion& q)
{
   // third row of rotation matrix R (body -> world)
   const float r31 = 2.0f * ((q.x * q.z) - (q.w * q.y));
   const float r32 = 2.0f * ((q.y * q.z) + (q.w * q.x));
   const float r33 = 1.0f - (2.0f * ((q.x * q.x) + (q.y * q.y)));

   return ((r31 * accel_body.x) + (r32 * accel_body.y) + (r33 * accel_body.z));
}

}   // namespace

namespace aeromight_control
{

AltitudeEkf::AltitudeEkf(const float process_noise_z,
                         const float process_noise_vz,
                         const float process_noise_accel_bias,
                         const float measurement_noise_baro,
                         const float tilt_gating_attitude_angle_rad,
                         const float tilt_gating_accel_weight)
    : m_process_noise_z{process_noise_z},
      m_process_noise_vz{process_noise_vz},
      m_process_noise_accel_bias{process_noise_accel_bias},
      m_measurement_noise_baro{measurement_noise_baro},
      m_tilt_gating_attitude_angle_rad{tilt_gating_attitude_angle_rad},
      m_tilt_gating_accel_weight{tilt_gating_accel_weight}

{
   reset();
}

void AltitudeEkf::predict(const math::Vector3&    accel_body_mps2,
                          const math::Quaternion& attitude,
                          const float             dt_s)
{
   math::Vector3 accel_body = accel_body_mps2;

   // remove bias
   accel_body.z -= m_state.a_z_bias;
   float accel_world_z = transform_accel_to_world_z(accel_body, attitude) + physics::constants::g_to_mps2;

   // apply tilt-compensation gate to accel_z
   const math::Vector3 euler = attitude.to_euler();
   if ((fabsf(euler.x) > m_tilt_gating_attitude_angle_rad) || (fabsf(euler.y) > m_tilt_gating_attitude_angle_rad))
   {
      accel_world_z = (fabsf(1.0f - m_tilt_gating_accel_weight) * m_last_accel_world_z) + (m_tilt_gating_accel_weight * accel_world_z);
   }

   // update state estimates
   // x^[n+1] = x^ + dt_s * f(x_dot,u) where x_dot is rate of change of state vector
   m_state.v_z += dt_s * accel_world_z;
   m_state.z += dt_s * m_state.v_z;

   m_last_accel_world_z = accel_world_z;

   // F_cont = [[0,1,0],[0,0,-r33],[0,0,0]]
   // discretize: F_d = I + dt_s * F_cont
   const float r33 = 1.0f - (2.0f * ((attitude.x * attitude.x) + (attitude.y * attitude.y)));
   m_f_d.set_identity();
   m_f_d.at(0, 1u)  = dt_s;
   m_f_d.at(1u, 2u) = -dt_s * r33;

   // initialize Q
   m_q.at(0, 0)   = dt_s * m_process_noise_z;
   m_q.at(1u, 1u) = dt_s * m_process_noise_vz;
   m_q.at(2u, 2u) = dt_s * m_process_noise_accel_bias;

   // 6) Covariance propagate: P = F_d * P * F_d^T + Q_d
   const auto f_d_transpose = m_f_d.transpose();
   const auto result_1      = m_f_d * m_p;                // F_d * P
   const auto result_2      = result_1 * f_d_transpose;   // (F_d * P) * F_d^T
   m_p                      = result_2 + m_q;             // (F_d * P * F_d^T) + Q_d

   // enforce symmetry and positive definite for numerical safety
   enfore_covariance_matrix_symmetry();
}

void AltitudeEkf::update(const float baro_altitude_m)
{
   if (!m_is_initialized)
   {
      m_state.z        = baro_altitude_m;
      m_is_initialized = true;
      return;
   }

   // Measurement model: x_meas = h(x)+v where v~N(0,R)
   // H = [1 0 0] (only altitude state is measured)
   // R = measurement noise covariance (scalar for 1D measurement)

   // S = HPH^T + R = P[0,0] + R
   const float p_0_0 = m_p.at(0, 0);
   const float s     = p_0_0 + m_measurement_noise_baro;

   // Kalman gain K = P * H^T / S = [P00; P10; P20] / S
   math::Matrix<float, num_state_dimensions, 1u> kalman_gain{};
   kalman_gain.at(0, 0) = m_p.at(0, 0) / s;
   kalman_gain.at(1, 0) = m_p.at(1u, 0) / s;
   kalman_gain.at(2, 0) = m_p.at(2u, 0) / s;

   // innovation
   const float y = baro_altitude_m - m_state.z;

   // state update i.e. x^[n+1] = x^[n] + K(x_meas - h(x_dot))
   m_state.z += kalman_gain.at(0, 0) * y;
   m_state.v_z += kalman_gain.at(1, 0) * y;
   m_state.a_z_bias += kalman_gain.at(2, 0) * y;

   // cov update (Joseph form): P = (I-KH) * P * (I-KH)^T + K * R * K^T
   math::Matrix<float, num_state_dimensions, num_state_dimensions> k_h{};   // KH = K * H -> 3x3
   // since H = [1 0 0], KH has first column = K and other cols zero
   for (size_t i = 0; i < num_state_dimensions; ++i)
   {
      k_h.at(i, 0) = kalman_gain.at(i, 0);
   }

   const auto i_minus_k_h           = m_identity - k_h;                   // (I-KH)
   const auto i_minus_k_h_transpose = i_minus_k_h.transpose();            // (I-KH)^T
   const auto result_1              = i_minus_k_h * m_p;                  // (I-KH) * P
   auto       p_new                 = result_1 * i_minus_k_h_transpose;   // (I-KH) * P * (I-KH)^T

   // K * R * K^T -> R (scalar) x  K * K^T
   const math::Matrix<float, num_state_dimensions, num_state_dimensions> result_2 = kalman_gain * kalman_gain.transpose();

   p_new += result_2 * m_measurement_noise_baro;

   // assign and enforce symmetry/diagonals
   m_p = p_new;
   enfore_covariance_matrix_symmetry();
}

void AltitudeEkf::enfore_covariance_matrix_symmetry()
{
   for (size_t i = 0; i < num_state_dimensions; ++i)
   {
      for (size_t j = 0; j < i; ++j)
      {
         m_p.at(j, i) = m_p.at(i, j);   // copy lower to upper (or average)
      }
      m_p.at(i, i) = std::max(m_p.at(i, i), min_recognizable_covariance);
   }
}

void AltitudeEkf::reset()
{
   m_state = EkfState{};

   // identity matrix
   m_identity.set_identity();

   // initialize P
   m_p.reset();
   m_p.at(0, 0)   = initial_uncertainty;
   m_p.at(1u, 1u) = initial_uncertainty;
   m_p.at(2u, 2u) = initial_uncertainty;

   m_q.reset();
   m_f_d.reset();

   m_is_initialized = false;
}

EkfState AltitudeEkf::get_state() const
{
   return m_state;
}

}   // namespace aeromight_control

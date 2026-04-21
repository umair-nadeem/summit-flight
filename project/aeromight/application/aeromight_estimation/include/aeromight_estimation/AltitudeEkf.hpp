#pragma once

#include "aeromight_estimation/AltitudeEkfParams.hpp"
#include "aeromight_estimation/EkfState.hpp"
#include "math/Matrix.hpp"
#include "math/Quaternion.hpp"

namespace aeromight_estimation
{

class AltitudeEkf
{
public:
   explicit AltitudeEkf(const AltitudeEkfParams& params);

   void predict(const math::Vec3f& accel_body_mps2, const math::Quaternion& attitude, const float dt_s);
   void update(const float baro_altitude_m);
   void reset();

   EkfState get_ekf_state() const;

private:
   void enfore_covariance_matrix_symmetry();

   static constexpr float       initial_uncertainty         = 10.0f;
   static constexpr float       min_recognizable_covariance = math::constants::epsilon;
   static constexpr std::size_t num_state_dimensions        = 3u;

   const AltitudeEkfParams&                                        m_params;
   EkfState                                                        m_state{};      // state vector
   math::Matrix<float, num_state_dimensions, num_state_dimensions> m_p{};          // P: covariance matrix
   math::Matrix<float, num_state_dimensions, num_state_dimensions> m_q{};          // Q: process noise
   math::Matrix<float, num_state_dimensions, num_state_dimensions> m_f_d{};        // F_d: Discretized Jacobian of f(x_dot,u)
   math::Matrix<float, num_state_dimensions, num_state_dimensions> m_identity{};   // F_d: Discretized Jacobian of f(x_dot,u)
   float                                                           m_last_accel_world_z{};
   bool                                                            m_is_initialized{false};
};

}   // namespace aeromight_estimation

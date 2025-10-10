#include "estimation/MadgwickFilter.hpp"

#include <algorithm>

#include "physics/constants.hpp"

namespace estimation
{

MadgwickFilter::MadgwickFilter(const float beta,
                               const float gyro_bias_gain,
                               const float accel_tolerance_mps2,
                               const float sampling_period_s)
    : m_beta{beta},
      m_gyro_bias_gain{gyro_bias_gain},
      m_accel_tolerance_mps2{accel_tolerance_mps2},
      m_sampling_period_s{sampling_period_s}
{
}

void MadgwickFilter::update(const physics::Vector3& accel_mps2, const physics::Vector3& gyro_radps)
{
   physics::Vector3       accel          = accel_mps2;
   const physics::Vector3 gyro_corrected = gyro_radps - m_gyro_bias;

   // get quaternion derivative from gyro
   physics::Quaternion q_dot_gyro = q_derivative(gyro_corrected);

   // perform accel gating to reject data during high dynamic situations
   if (std::abs(accel.get_norm() - physics::constants::g_to_mps2) <= m_accel_tolerance_mps2)
   {
      accel.normalize();
      const auto& q  = m_quaternion;
      // Objective function and Jacobian (from Madgwick's paper)
      const float f1 = 2.0f * (q.x * q.z - q.w * q.y) - accel.x;
      const float f2 = 2.0f * (q.w * q.x + q.y * q.z) - accel.y;
      const float f3 = 2.0f * (0.5f - q.x * q.x - q.y * q.y) - accel.z;

      physics::Quaternion gradient{-2.0f * q.y * f1 + 2.0f * q.x * f2,
                                   2.0f * q.z * f1 + 2.0f * q.w * f2 - 4.0f * q.x * f3,
                                   -2.0f * q.w * f1 + 2.0f * q.z * f2 - 4.0f * q.y * f3,
                                   2.0f * q.x * f1 + 2.0f * q.y * f2};
      if (gradient.get_norm() > min_recognizable_gradient)
      {
         gradient.normalize();
      }
      else
      {
         gradient = physics::Quaternion{0.0f, 0.0f, 0.0f, 0.0f};
      }

      // remove error from estimated rotation change
      q_dot_gyro -= gradient * m_beta;

      // update gyro bias
      m_gyro_bias -= (physics::Vector3{gradient.x, gradient.y, gradient.z} * m_gyro_bias_gain * m_sampling_period_s);
      m_gyro_bias.x = std::clamp(m_gyro_bias.x, -max_gyro_bias_radps, max_gyro_bias_radps);
      m_gyro_bias.y = std::clamp(m_gyro_bias.y, -max_gyro_bias_radps, max_gyro_bias_radps);
      m_gyro_bias.z = std::clamp(m_gyro_bias.z, -max_gyro_bias_radps, max_gyro_bias_radps);
   }

   // update quaternion
   m_quaternion += (q_dot_gyro * m_sampling_period_s);
   m_quaternion.normalize();
}

physics::Quaternion MadgwickFilter::q_derivative(const physics::Vector3& gyro) const
{
   const auto& q = m_quaternion;
   return physics::Quaternion{
       0.5f * (-q.x * gyro.x - q.y * gyro.y - q.z * gyro.z),   // w_dot
       0.5f * (q.w * gyro.x + q.y * gyro.z - q.z * gyro.y),    // x_dot
       0.5f * (q.w * gyro.y - q.x * gyro.z + q.z * gyro.x),    // y_dot
       0.5f * (q.w * gyro.z + q.x * gyro.y - q.y * gyro.x)     // z_dot
   };
}

void MadgwickFilter::reset_orientation()
{
   m_quaternion = physics::Quaternion{};
}

void MadgwickFilter::reset_gyro_bias()
{
   m_gyro_bias = physics::Vector3{};
}

physics::Quaternion MadgwickFilter::get_quaternion() const
{
   return m_quaternion;
}

physics::Vector3 MadgwickFilter::get_gyro_bias() const
{
   return m_gyro_bias;
}

physics::Vector3 MadgwickFilter::get_unbiased_gyro_data(const physics::Vector3& raw_gyro) const
{
   return physics::Vector3{raw_gyro - m_gyro_bias};
}

}   // namespace estimation

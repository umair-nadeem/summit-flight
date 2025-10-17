#include "estimation/MadgwickFilter.hpp"

#include <algorithm>

#include "physics/constants.hpp"

namespace estimation
{

MadgwickFilter::MadgwickFilter(const float beta,
                               const float gyro_bias_gain,
                               const float accel_tolerance_mps2)
    : m_beta{beta},
      m_gyro_bias_gain{gyro_bias_gain},
      m_accel_tolerance_mps2{accel_tolerance_mps2}
{
}

void MadgwickFilter::update_in_ned_frame(const math::Vector3& accel_mps2, const math::Vector3& gyro_radps, const float dt_s)
{
   math::Vector3       accel          = accel_mps2;
   const math::Vector3 gyro_corrected = gyro_radps - m_gyro_bias;

   // get quaternion derivative from gyro
   math::Quaternion q_dot_gyro = q_derivative(gyro_corrected);

   // perform accel gating to reject data during high dynamic situations
   if (std::abs(accel.get_norm() - physics::constants::g_to_mps2) <= m_accel_tolerance_mps2)
   {
      accel.normalize();

      const auto&                           q      = m_quaternion;
      const std::tuple<float, float, float> result = get_jacobian_ned_frame(q, accel);

      const float f1 = std::get<0>(result);
      const float f2 = std::get<1>(result);
      const float f3 = std::get<2>(result);

      math::Quaternion gradient{-2.0f * q.y * f1 + 2.0f * q.x * f2,
                                2.0f * q.z * f1 + 2.0f * q.w * f2 - 4.0f * q.x * f3,
                                -2.0f * q.w * f1 + 2.0f * q.z * f2 - 4.0f * q.y * f3,
                                2.0f * q.x * f1 + 2.0f * q.y * f2};
      if (gradient.get_norm() > min_recognizable_gradient)
      {
         gradient.normalize();
      }
      else
      {
         gradient = math::Quaternion{0.0f, 0.0f, 0.0f, 0.0f};
      }

      // remove error from estimated rotation change
      q_dot_gyro -= gradient * m_beta;

      // update gyro bias
      m_gyro_bias -= (math::Vector3{gradient.x, gradient.y, gradient.z} * m_gyro_bias_gain * dt_s);
      m_gyro_bias.x = std::clamp(m_gyro_bias.x, -max_gyro_bias_radps, max_gyro_bias_radps);
      m_gyro_bias.y = std::clamp(m_gyro_bias.y, -max_gyro_bias_radps, max_gyro_bias_radps);
      m_gyro_bias.z = std::clamp(m_gyro_bias.z, -max_gyro_bias_radps, max_gyro_bias_radps);
   }

   // update quaternion
   m_quaternion += (q_dot_gyro * dt_s);
   m_quaternion.normalize();
}

void MadgwickFilter::update_in_enu_frame(const math::Vector3& accel_mps2, const math::Vector3& gyro_radps, const float dt_s)
{
   math::Vector3       accel          = accel_mps2;
   const math::Vector3 gyro_corrected = gyro_radps - m_gyro_bias;

   // get quaternion derivative from gyro
   math::Quaternion q_dot_gyro = q_derivative(gyro_corrected);

   // perform accel gating to reject data during high dynamic situations
   if (std::abs(accel.get_norm() - physics::constants::g_to_mps2) <= m_accel_tolerance_mps2)
   {
      accel.normalize();

      const auto&                           q      = m_quaternion;
      const std::tuple<float, float, float> result = get_jacobian_enu_frame(q, accel);

      const float f1 = std::get<0>(result);
      const float f2 = std::get<1>(result);
      const float f3 = std::get<2>(result);

      math::Quaternion gradient{-2.0f * q.y * f1 + 2.0f * q.x * f2,
                                2.0f * q.z * f1 + 2.0f * q.w * f2 - 4.0f * q.x * f3,
                                -2.0f * q.w * f1 + 2.0f * q.z * f2 - 4.0f * q.y * f3,
                                2.0f * q.x * f1 + 2.0f * q.y * f2};
      if (gradient.get_norm() > min_recognizable_gradient)
      {
         gradient.normalize();
      }
      else
      {
         gradient = math::Quaternion{0.0f, 0.0f, 0.0f, 0.0f};
      }

      // remove error from estimated rotation change
      q_dot_gyro -= gradient * m_beta;

      // update gyro bias
      m_gyro_bias -= (math::Vector3{gradient.x, gradient.y, gradient.z} * m_gyro_bias_gain * dt_s);
      m_gyro_bias.x = std::clamp(m_gyro_bias.x, -max_gyro_bias_radps, max_gyro_bias_radps);
      m_gyro_bias.y = std::clamp(m_gyro_bias.y, -max_gyro_bias_radps, max_gyro_bias_radps);
      m_gyro_bias.z = std::clamp(m_gyro_bias.z, -max_gyro_bias_radps, max_gyro_bias_radps);
   }

   // update quaternion
   m_quaternion += (q_dot_gyro * dt_s);
   m_quaternion.normalize();
}

math::Quaternion MadgwickFilter::q_derivative(const math::Vector3& gyro) const
{
   const auto& q = m_quaternion;
   return math::Quaternion{
       0.5f * (-q.x * gyro.x - q.y * gyro.y - q.z * gyro.z),   // w_dot
       0.5f * (q.w * gyro.x + q.y * gyro.z - q.z * gyro.y),    // x_dot
       0.5f * (q.w * gyro.y - q.x * gyro.z + q.z * gyro.x),    // y_dot
       0.5f * (q.w * gyro.z + q.x * gyro.y - q.y * gyro.x)     // z_dot
   };
}

void MadgwickFilter::reset()
{
   m_quaternion = math::Quaternion{};
   m_gyro_bias  = math::Vector3{};
}

math::Quaternion MadgwickFilter::get_quaternion() const
{
   return m_quaternion;
}

math::Vector3 MadgwickFilter::get_gyro_bias() const
{
   return m_gyro_bias;
}

math::Vector3 MadgwickFilter::get_unbiased_gyro_data(const math::Vector3& raw_gyro) const
{
   return math::Vector3{raw_gyro - m_gyro_bias};
}

}   // namespace estimation

#include "estimation/AttitudeEstimator.hpp"

#include <algorithm>

namespace estimation
{

AttitudeEstimator::AttitudeEstimator(const float accelerometer_weight,
                                     const float gyro_bias_weight)
    : m_accelerometer_weight{accelerometer_weight},
      m_gyro_bias_weight{gyro_bias_weight}
{
}

void AttitudeEstimator::update(const math::Vector3& accel_mps2, const math::Vector3& gyro_radps, const float dt_s)
{
   using namespace math;

   Quaternion q_last = m_q;

   // Angular rate of correction
   Vector3     corr;
   const float spinRate = gyro_radps.norm();

   m_q.normalize();

   // Accelerometer correction
   // Project 'k' unit vector of earth frame to body frame
   // Optimized version with dropped zeros
   Vector3 k{
       2.0f * (m_q.x * m_q.z - m_q.w * m_q.y),
       2.0f * (m_q.y * m_q.z + m_q.w * m_q.x),
       (m_q.w * m_q.w - m_q.x * m_q.x - m_q.y * m_q.y + m_q.z * m_q.z)};

   const float accel_norm_sq = accel_mps2.norm_squared();

   if ((accel_norm_sq > lower_accel_limit * lower_accel_limit) &&
       (accel_norm_sq < upper_accel_limit * upper_accel_limit))
   {
      const Vector3 accel_dir = accel_mps2.normalized();
      corr += (k.cross(accel_dir)) * m_accelerometer_weight;
   }

   // Gyro bias estimation
   if (spinRate < max_gyro_spin_rate)
   {
      m_gyro_bias += corr * (m_gyro_bias_weight * dt_s);

      m_gyro_bias[0] = std::clamp(m_gyro_bias[0], -max_gyro_bias_radps, max_gyro_bias_radps);
      m_gyro_bias[1] = std::clamp(m_gyro_bias[1], -max_gyro_bias_radps, max_gyro_bias_radps);
      m_gyro_bias[2] = std::clamp(m_gyro_bias[2], -max_gyro_bias_radps, max_gyro_bias_radps);
   }

   m_rates = gyro_radps + m_gyro_bias;

   // Feed forward gyro
   corr += m_rates;

   // Apply correction to state
   m_q += m_q.derivative1(corr) * dt_s;

   m_q.normalize();

   if (!m_q.is_all_finite())
   {
      // Reset quaternion to last good state
      m_q = q_last;
      m_rates.zero();
      m_gyro_bias.zero();
   }
}

void AttitudeEstimator::reset()
{
   m_q = math::Quaternion{};
   m_gyro_bias.zero();
   m_rates.zero();
}

math::Quaternion AttitudeEstimator::get_quaternion() const
{
   return m_q;
}

math::Vector3 AttitudeEstimator::get_gyro_bias() const
{
   return m_gyro_bias;
}

math::Vector3 AttitudeEstimator::get_unbiased_gyro_data(const math::Vector3& raw_gyro) const
{
   return math::Vector3{raw_gyro - m_gyro_bias};
}

}   // namespace estimation

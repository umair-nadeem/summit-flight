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

void AttitudeEstimator::update(const math::Vec3f& accel_mps2, const math::Vec3f& gyro_radps, const float dt_s)
{
   using namespace math;

   Quaternion q_last = m_q;

   // Angular rate of correction
   Vec3f       corr{};
   const float spinRate = gyro_radps.norm();

   m_q.normalize();

   // Accelerometer correction
   // Project 'k' unit vector of earth frame to body frame
   // Optimized version with dropped zeros
   Vec3f k{
       2.0f * (m_q[1] * m_q[3] - m_q[0] * m_q[2]),
       2.0f * (m_q[2] * m_q[3] + m_q[0] * m_q[1]),
       (m_q[0] * m_q[0] - m_q[1] * m_q[1] - m_q[2] * m_q[2] + m_q[3] * m_q[3])};

   const float accel_norm_sq = accel_mps2.norm_squared();

   if ((accel_norm_sq > lower_accel_limit * lower_accel_limit) &&
       (accel_norm_sq < upper_accel_limit * upper_accel_limit))
   {
      const Vec3f accel_dir = accel_mps2.normalized();
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

math::Vec3f AttitudeEstimator::get_gyro_bias() const
{
   return m_gyro_bias;
}

math::Vec3f AttitudeEstimator::get_unbiased_gyro_data(const math::Vec3f& raw_gyro) const
{
   return math::Vec3f{raw_gyro - m_gyro_bias};
}

}   // namespace estimation

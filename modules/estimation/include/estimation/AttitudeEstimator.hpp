#pragma once

#include <cstdint>
#include <tuple>

#include "estimation/AttitudeEstimatorParams.hpp"
#include "math/Quaternion.hpp"
#include "math/Vector3.hpp"

namespace estimation
{

class AttitudeEstimator
{
public:
   explicit AttitudeEstimator(const AttitudeEstimatorParams& params);

   void update(const math::Vec3f& accel_mps2, const math::Vec3f& gyro_radps, const float dt_s);
   void reset();

   math::Quaternion get_quaternion() const;
   math::Vec3f      get_gyro_bias() const;
   math::Vec3f      get_unbiased_gyro_data(const math::Vec3f& raw_gyro) const;

private:
   static constexpr float max_gyro_bias_radps = 0.1f;
   static constexpr float max_gyro_spin_rate  = 0.175f;
   static constexpr float upper_accel_limit   = math::constants::g_to_mps2 * 1.1f;
   static constexpr float lower_accel_limit   = math::constants::g_to_mps2 * 0.9f;

   const AttitudeEstimatorParams& m_params;
   math::Quaternion               m_q{};
   math::Vec3f                    m_gyro_bias{};
   math::Vec3f                    m_rates{};
};

}   // namespace estimation

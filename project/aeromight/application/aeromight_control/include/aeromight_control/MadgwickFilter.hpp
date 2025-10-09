#pragma once

#include <cstdint>

#include "physics/Quaternion.hpp"
#include "physics/Vector3.hpp"

namespace aeromight_control
{

class MadgwickFilter
{
public:
   explicit MadgwickFilter(const float beta, const float gyro_bias_gain, const float accel_tolerance_mps2, const float sampling_period_s);

   void update(const physics::Vector3& accel_mps2, const physics::Vector3& gyro_radps);
   void reset_orientation();
   void reset_gyro_bias();

   physics::Quaternion get_quaternion() const;
   physics::Vector3    get_gyro_bias() const;
   physics::Vector3    get_unbiased_gyro_data(const physics::Vector3& raw_gyro) const;

private:
   physics::Quaternion q_derivative(const physics::Vector3& gyro) const;

   static constexpr float max_gyro_bias_radps       = 0.1f;
   static constexpr float g_to_mps2                 = 9.80665f;
   static constexpr float min_recognizable_gradient = 1e-9f;

   const float         m_beta;
   const float         m_gyro_bias_gain;
   const float         m_accel_tolerance_mps2;
   const float         m_sampling_period_s;
   physics::Quaternion m_quaternion{};
   physics::Vector3    m_gyro_bias{};
   uint32_t            m_last_timestamp_gyro_data_ms{0};
};

}   // namespace aeromight_control

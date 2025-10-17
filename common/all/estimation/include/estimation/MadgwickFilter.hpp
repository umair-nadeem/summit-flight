#pragma once

#include <cstdint>
#include <tuple>

#include "math/Quaternion.hpp"
#include "math/Vector3.hpp"

namespace estimation
{

class MadgwickFilter
{
public:
   explicit MadgwickFilter(const float beta, const float gyro_bias_gain, const float accel_tolerance_mps2);

   void update_in_ned_frame(const math::Vector3& accel_mps2, const math::Vector3& gyro_radps, const float dt_s);
   void update_in_enu_frame(const math::Vector3& accel_mps2, const math::Vector3& gyro_radps, const float dt_s);
   void reset();

   math::Quaternion get_quaternion() const;
   math::Vector3    get_gyro_bias() const;
   math::Vector3    get_unbiased_gyro_data(const math::Vector3& raw_gyro) const;

private:
   void             update();
   math::Quaternion q_derivative(const math::Vector3& gyro) const;

   static constexpr std::tuple<float, float, float> get_jacobian_ned_frame(const auto& q, const auto& accel)
   {
      const float f1 = 2.0f * (q.x * q.z - q.w * q.y) - accel.x;
      const float f2 = 2.0f * (q.w * q.x + q.y * q.z) - accel.y;
      const float f3 = 2.0f * (0.5f - q.x * q.x - q.y * q.y) + accel.z;
      return std::make_tuple(f1, f2, f3);
   }

   static constexpr std::tuple<float, float, float> get_jacobian_enu_frame(const auto& q, const auto& accel)
   {
      const float f1 = 2.0f * (q.x * q.z - q.w * q.y) - accel.x;
      const float f2 = 2.0f * (q.w * q.x + q.y * q.z) - accel.y;
      const float f3 = 2.0f * (0.5f - q.x * q.x - q.y * q.y) - accel.z;
      return std::make_tuple(f1, f2, f3);
   }

   static constexpr float max_gyro_bias_radps       = 0.1f;
   static constexpr float min_recognizable_gradient = 1e-9f;

   const float      m_beta;
   const float      m_gyro_bias_gain;
   const float      m_accel_tolerance_mps2;
   math::Quaternion m_quaternion{};
   math::Vector3    m_gyro_bias{};
};

}   // namespace estimation

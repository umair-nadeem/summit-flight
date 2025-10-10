#include "aeromight_control/AltitudeEkf.hpp"

namespace aeromight_control
{

AltitudeEkf::AltitudeEkf(const float process_noise_z,
                         const float process_noise_vz,
                         const float process_noise_accel_bias,
                         const float measurement_noise_baro,
                         const float sampling_period_s)
    : m_process_noise_z{process_noise_z},
      m_process_noise_vz{process_noise_vz},
      m_process_noise_accel_bias{process_noise_accel_bias},
      m_measurement_noise_baro{measurement_noise_baro},
      m_sampling_period_s{sampling_period_s}
{
}

void AltitudeEkf::predict(const physics::Vector3& accel_body_mps2, const physics::Quaternion& attitude_quaternion)
{
   // physics::Vector3 accel_body = accel_body_mps2;
   // accel_body.z -= m_state.accel_z_bias;

   // remove bias
   // const float accel_world_z = transform_accel_to_world_z(accel_body, attitude_quaternion);
}

// constexpr float AltitudeEkf::transform_accel_to_world_z(const physics::Vector3& accel_body, const physics::Quaternion& q)
// {
//    // third row of rotation matrix R (body -> world)
//    const float r31 = 2.0f * (q.x * q.z - q.w * q.y);
//    const float r32 = 2.0f * (q.y * q.z + q.w * q.x);
//    const float r33 = 1.0f - (2.0f * (q.x * q.x + q.y * q.y));

//    return ((r31 * accel_body.x) + (r32 * accel_body.y) + (r33 * accel_body.z)) - physics::constants::g_to_mps2;
// }

EkfState AltitudeEkf::get_state() const
{
   return m_state;
}

}   // namespace aeromight_control

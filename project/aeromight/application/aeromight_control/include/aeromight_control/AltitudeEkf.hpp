#pragma once

#include "aeromight_control/EkfState.hpp"
#include "physics/Quaternion.hpp"

namespace aeromight_control
{

class AltitudeEkf
{
public:
   explicit AltitudeEkf(const float process_noise_z,
                        const float process_noise_vz,
                        const float process_noise_accel_bias,
                        const float measurement_noise_baro,
                        const float sampling_period_s);

   static void predict(const physics::Vector3& accel_body_mps2, const physics::Quaternion& attitude_quaternion);

   EkfState get_state() const;

private:
   // static constexpr float transform_accel_to_world_z(const physics::Vector3& accel_body, const physics::Quaternion& q);

   const float m_process_noise_z;
   const float m_process_noise_vz;
   const float m_process_noise_accel_bias;
   const float m_measurement_noise_baro;
   const float m_sampling_period_s;
   EkfState    m_state{};
};

}   // namespace aeromight_control

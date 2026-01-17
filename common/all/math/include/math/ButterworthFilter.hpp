#pragma once

#include "error/error_handler.hpp"
#include "physics/constants.hpp"

namespace math
{

// Second-order LPF
class ButterworthFilter
{
public:
   explicit ButterworthFilter(const float cutoff_frequency_hz,
                              const float nominal_dt_s)
       : m_cutoff_frequency_hz{cutoff_frequency_hz},
         m_sampling_frequency_hz{1.0f / nominal_dt_s},
         m_nominal_dt_s{nominal_dt_s}
   {
      error::verify(m_cutoff_frequency_hz < (m_sampling_frequency_hz / 2.0f));

      reset();
      update_coefficients(m_nominal_dt_s);
   }

   float apply(const float sample)
   {
      // Direct Form II implementation
      const float delay_element_0 = sample - (m_delay_element_1 * m_a1) - (m_delay_element_2 * m_a2);
      const float output          = (delay_element_0 * m_b0) + (m_delay_element_1 * m_b1) + (m_delay_element_2 * m_b2);

      m_delay_element_2 = m_delay_element_1;
      m_delay_element_1 = delay_element_0;

      return output;
   }

   void reset(const float value = 0.0f)
   {
      m_delay_element_1 = value;
      m_delay_element_2 = value;
   }

private:
   void update_coefficients(const float dt_s)
   {
      static constexpr float sqrt2                 = 1.41421356237f;
      const float            sampling_frequency_hz = 1.0f / dt_s;
      const float            omega                 = tanf(physics::constants::pi * m_cutoff_frequency_hz / sampling_frequency_hz);
      const float            omega2                = omega * omega;
      const float            transfer_function     = 1.0f / (1.f + (sqrt2 * omega) + omega2);

      m_b0 = omega2 * transfer_function;
      m_b1 = 2.f * m_b0;
      m_b2 = m_b0;

      m_a1 = 2.f * (omega2 - 1.f) * transfer_function;
      m_a2 = (1.f - (sqrt2 * omega) + omega2) * transfer_function;
   }

   const float m_cutoff_frequency_hz;
   const float m_sampling_frequency_hz;
   const float m_nominal_dt_s;
   float       m_a1{0.0f};
   float       m_a2{0.0f};
   float       m_b0{0.0f};
   float       m_b1{0.0f};
   float       m_b2{0.0f};
   float       m_delay_element_1{0.0f};
   float       m_delay_element_2{0.0f};
};

}   // namespace math

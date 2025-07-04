#pragma once

#include <cstddef>

namespace aeromight_sensors
{

template <typename Led>
class SensorAcquisition
{
public:
   explicit SensorAcquisition(Led& led, const size_t period)
       : m_led{led},
         m_period_in_ms{period}
   {
   }

   void run_once()
   {
      size_t delay_multiplier = min_delay_multiplier;
      bool   increasing_delay = true;

      // m_led.toggle();
      // add delay

      increasing_delay ? ++delay_multiplier : --delay_multiplier;

      // max limit reached, oscillate downwards
      if (delay_multiplier >= max_delay_multiplier)
      {
         delay_multiplier = max_delay_multiplier;
         increasing_delay = false;
      }

      if (delay_multiplier <= min_delay_multiplier)
      {
         delay_multiplier = min_delay_multiplier;
         increasing_delay = true;
      }
   }

   size_t get_period_ms() const
   {
      return m_period_in_ms;
   }

private:
   static constexpr size_t blocking_delay       = 500u;
   static constexpr size_t min_delay_multiplier = 1u;
   static constexpr size_t max_delay_multiplier = 10;

   Led&         m_led;
   const size_t m_period_in_ms;
};

}   // namespace aeromight_sensors

#pragma once

#include <cstddef>

#include "error/error_handler.h"

namespace aeromight_sensors
{

template <typename Led>
class SensorAcquisition
{
public:
   explicit SensorAcquisition(Led& led, const size_t period_in_ms)
       : m_led{led},
         m_period_in_ms{period_in_ms}
   {
      error::assert(m_period_in_ms > 0);
   }

   void run_once()
   {
      if (m_led_on)
      {
         m_on_time_counter += m_period_in_ms;

         if (m_on_time_counter >= m_target_on_time)
         {
            m_led.set_low();
            m_led_on           = false;
            m_on_time_counter  = 0u;
            m_off_time_counter = 0u;

            // adjust target_on_time for next time
            m_target_on_time += on_time_increment_delta;

            if (m_target_on_time > max_on_time)
            {
               m_target_on_time = min_on_time;
            }
         }
      }
      else
      {
         m_off_time_counter += m_period_in_ms;

         if (m_off_time_counter >= fixed_off_time)
         {
            m_led.set_high();
            m_led_on           = true;
            m_on_time_counter  = 0u;
            m_off_time_counter = 0u;
         }
      }
   }

   size_t get_period_ms() const
   {
      return m_period_in_ms;
   }

private:
   static constexpr size_t on_time_increment_delta = 500u;
   static constexpr size_t min_on_time             = 500u;
   static constexpr size_t max_on_time             = 4000u;
   static constexpr size_t fixed_off_time          = 1000u;

   Led&         m_led;
   const size_t m_period_in_ms;
   size_t       m_target_on_time{min_on_time};
   size_t       m_on_time_counter{0u};
   size_t       m_off_time_counter{0u};
   bool         m_led_on{false};
};

}   // namespace aeromight_sensors

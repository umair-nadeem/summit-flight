#pragma once

#include <cstddef>

#include "error/error_handler.hpp"

namespace aeromight_sensors
{

template <typename Led, typename Logger>
class SensorAcquisition
{
public:
   explicit SensorAcquisition(Led& led, Logger& logger, const std::size_t period_in_ms)
       : m_led{led},
         m_logger{logger},
         m_period_in_ms{period_in_ms}
   {
      error::verify(m_period_in_ms > 0);
      m_logger.enable();
   }

   void run_once()
   {
      blink_led();
   }

   std::size_t get_period_ms() const
   {
      return m_period_in_ms;
   }

private:
   void blink_led()
   {
      m_led_state_duration_counter += m_period_in_ms;
      if (m_led_state_duration_counter >= led_state_duration)
      {
         if (m_led_on)
         {
            m_led.set_low();
            m_led_on = false;
            m_logger.print("123-off");
         }
         else
         {
            m_led.set_high();
            m_led_on = true;
         }

         m_led_state_duration_counter = 0;
      }
   }

   static constexpr std::size_t led_state_duration = 2000u;

   Led&              m_led;
   Logger&           m_logger;
   const std::size_t m_period_in_ms;
   std::size_t       m_led_state_duration_counter{0u};
   bool              m_led_on{false};
};

}   // namespace aeromight_sensors

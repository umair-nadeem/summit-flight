#pragma once

#include "interfaces/IClockSource.hpp"

namespace led
{

template <typename T, interfaces::IClockSource ClockSource>
class Led
{
public:
   explicit Led(T& led)
       : m_led{led}
   {
   }

   void turn_on()
   {
      m_led.turn_on();
   }

   void turn_off()
   {
      m_led.turn_off();
   }

   void toggle(const uint32_t period)
   {
      const uint32_t now_ms = ClockSource::now_ms();

      if ((now_ms - m_led_timer) >= period)
      {
         if (m_led_is_on)
         {
            m_led.turn_off();
            m_led_is_on = false;
            m_led_timer = now_ms;
         }
         else
         {
            m_led.turn_on();
            m_led_is_on = true;
            m_led_timer = now_ms;
         }
      }
   }

private:
   T&       m_led;
   uint32_t m_led_timer{0};
   bool     m_led_is_on{false};
};

}   // namespace led

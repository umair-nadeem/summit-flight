#pragma once

#include <cstddef>

#include "error/error_handler.hpp"
#include "interfaces/pcb_component/ILed.hpp"
#include "interfaces/peripherals/IImuSensorDriver.hpp"

namespace aeromight_sensors
{

template <interfaces::peripherals::IImuSensorDriver Mpu6500Driver, interfaces::pcb_component::ILed Led, typename Logger>
class SensorAcquisition
{
public:
   explicit SensorAcquisition(Mpu6500Driver& mpu6500_driver, Led& led, Logger& logger, const std::size_t period_in_ms)
       : m_mpu6500_driver{mpu6500_driver},
         m_led{led},
         m_logger{logger},
         m_period_in_ms{period_in_ms}
   {
      error::verify(m_period_in_ms > 0);
      m_logger.enable();
   }

   void run_once()
   {
      m_mpu6500_driver.execute();

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
            m_led.turn_off();
            m_led_on = false;
            m_logger.print("123-off");
         }
         else
         {
            m_led.turn_on();
            m_led_on = true;
         }

         m_led_state_duration_counter = 0;
      }
   }

   static constexpr std::size_t led_state_duration = 2000u;

   Mpu6500Driver&    m_mpu6500_driver;
   Led&              m_led;
   Logger&           m_logger;
   const std::size_t m_period_in_ms;
   std::size_t       m_led_state_duration_counter{0u};
   bool              m_led_on{false};
};

}   // namespace aeromight_sensors

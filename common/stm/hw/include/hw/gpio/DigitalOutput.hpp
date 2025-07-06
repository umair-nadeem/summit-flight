#pragma once

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

namespace hw::gpio
{

class DigitalOutput
{
public:
   explicit DigitalOutput(GPIO_TypeDef* const port, const uint16_t pin, const bool active_low = false)
       : m_port{port},
         m_pin{pin},
         m_active_low{active_low}
   {
   }

   void set_high()
   {
      m_active_low ? write(GPIO_PinState::GPIO_PIN_RESET) : write(GPIO_PinState::GPIO_PIN_SET);
   }

   void set_low()
   {
      m_active_low ? write(GPIO_PinState::GPIO_PIN_SET) : write(GPIO_PinState::GPIO_PIN_RESET);
   }

   bool is_high() const
   {
      return (m_port->ODR & m_pin);
   }

   bool is_low() const
   {
      return !is_high();
   }

private:
   void write(const GPIO_PinState state)
   {
      HAL_GPIO_WritePin(m_port, m_pin, state);
   }

   GPIO_TypeDef* const m_port;
   const uint16_t      m_pin;
   const bool          m_active_low;
};

}   // namespace hw::gpio

#pragma once

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

namespace hw_abstraction
{

class DigitalOutput
{
public:
   explicit DigitalOutput(GPIO_TypeDef* const port, const uint16_t pin)
       : m_port{port},
         m_pin{pin}
   {
      turn_off();
   }

   void turn_on() const
   {
      HAL_GPIO_WritePin(m_port, m_pin, GPIO_PinState::GPIO_PIN_SET);
   }

   void turn_off() const
   {
      HAL_GPIO_WritePin(m_port, m_pin, GPIO_PinState::GPIO_PIN_RESET);
   }

private:
   GPIO_TypeDef* const m_port;
   const uint16_t      m_pin;
};

}   // namespace hw_abstraction

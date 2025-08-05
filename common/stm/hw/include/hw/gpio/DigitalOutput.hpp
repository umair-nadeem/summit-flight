#pragma once

#include "interfaces/IDigitalOutput.hpp"
#include "stm32f4xx_ll_gpio.h"

namespace hw::gpio
{

class DigitalOutput
{
public:
   explicit DigitalOutput(GPIO_TypeDef* const port, const uint32_t pin, const bool active_low = false)
       : m_port{port},
         m_pin{pin},
         m_active_low{active_low}
   {
   }

   void set_high()
   {
      if (m_active_low)
      {
         LL_GPIO_ResetOutputPin(m_port, m_pin);
      }
      else
      {
         LL_GPIO_SetOutputPin(m_port, m_pin);
      }
   }

   void set_low()
   {
      if (m_active_low)
      {
         LL_GPIO_SetOutputPin(m_port, m_pin);
      }
      else
      {
         LL_GPIO_ResetOutputPin(m_port, m_pin);
      }
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
   GPIO_TypeDef* const m_port;
   const uint32_t      m_pin;
   const bool          m_active_low;
};

static_assert(interfaces::IDigitalOutput<DigitalOutput>);

}   // namespace hw::gpio

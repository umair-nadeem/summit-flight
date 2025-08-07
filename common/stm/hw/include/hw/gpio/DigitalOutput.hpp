#pragma once

#include "hw/HwPin.hpp"
#include "interfaces/IDigitalOutput.hpp"

namespace hw::gpio
{

class DigitalOutput
{
public:
   explicit DigitalOutput(const HwPin& hw_pin, const bool active_low = false)
       : m_hw_pin{hw_pin},
         m_active_low{active_low}
   {
   }

   void set_high()
   {
      if (m_active_low)
      {
         LL_GPIO_ResetOutputPin(m_hw_pin.port, m_hw_pin.pin);
      }
      else
      {
         LL_GPIO_SetOutputPin(m_hw_pin.port, m_hw_pin.pin);
      }
   }

   void set_low()
   {
      if (m_active_low)
      {
         LL_GPIO_SetOutputPin(m_hw_pin.port, m_hw_pin.pin);
      }
      else
      {
         LL_GPIO_ResetOutputPin(m_hw_pin.port, m_hw_pin.pin);
      }
   }

   bool is_high() const
   {
      return (m_hw_pin.port->ODR & m_hw_pin.pin);
   }

   bool is_low() const
   {
      return !is_high();
   }

private:
   const HwPin& m_hw_pin;
   const bool   m_active_low;
};

static_assert(interfaces::IDigitalOutput<DigitalOutput>);

}   // namespace hw::gpio

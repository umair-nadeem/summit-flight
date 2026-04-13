#pragma once

#include "hw/HwPin.hpp"
#include "interfaces/pcb_component/ILed.hpp"

namespace hw::pcb_component
{

class Led
{
public:
   explicit Led(const HwPin& hw_pin, const bool active_low = false)
       : m_hw_pin{hw_pin},
         m_active_low{active_low}
   {
   }

   void turn_on()
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

   void turn_off()
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

private:
   const HwPin& m_hw_pin;
   const bool   m_active_low;
};

static_assert(interfaces::pcb_component::ILed<Led>);
}   // namespace hw::pcb_component

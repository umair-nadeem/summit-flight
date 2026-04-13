#pragma once

#include "interfaces/hw/IDigitalOutput.hpp"
#include "interfaces/pcb_component/IEnabler.hpp"

namespace hw::pcb_component
{

template <interfaces::hw::IDigitalOutput DigitalOuput>
class Enabler
{
public:
   explicit Enabler(DigitalOuput& digital_output)
       : m_digital_output{digital_output}
   {
   }

   void enable()
   {
      m_digital_output.set_high();
   }

   void disable()
   {
      m_digital_output.set_low();
   }

   bool is_enabled()
   {
      return m_digital_output.is_high();
   }

private:
   DigitalOuput& m_digital_output;

   static_assert(interfaces::pcb_component::IEnabler<Enabler<DigitalOuput>>);
};

}   // namespace hw::pcb_component

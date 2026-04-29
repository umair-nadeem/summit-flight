#pragma once

namespace sitl::pcb_component
{

struct Led
{
   bool m_state{false};

   void turn_on()
   {
      m_state = true;
   }

   void turn_off()
   {
      m_state = false;
   }
};

}   // namespace sitl::pcb_component

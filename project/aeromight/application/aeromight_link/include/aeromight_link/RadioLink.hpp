#pragma once

#include <cstdint>

namespace aeromight_link
{

template <typename RadioReceiver>
class RadioLink
{
public:
   explicit RadioLink(RadioReceiver& radio_receiver,
                      const uint32_t period_in_ms)
       : m_radio_receiver{radio_receiver},
         m_period_in_ms(period_in_ms)
   {
   }

   void run_once()
   {
      m_radio_receiver.execute();
   }

   uint32_t get_period_ms() const
   {
      return m_period_in_ms;
   }

private:
   RadioReceiver& m_radio_receiver;
   const uint32_t m_period_in_ms;
};

}   // namespace aeromight_link

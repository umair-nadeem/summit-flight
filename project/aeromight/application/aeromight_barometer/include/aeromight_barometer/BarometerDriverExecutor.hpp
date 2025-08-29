#pragma once

#include "error/error_handler.hpp"
#include "interfaces/peripherals/ISensorDriver.hpp"

namespace aeromight_barometer
{

template <interfaces::peripherals::ISensorDriver Bmp390>
class BarometerDriverExecutor
{
public:
   explicit BarometerDriverExecutor(Bmp390&           bmp390,
                                    const std::size_t period_in_ms)
       : m_bmp390{bmp390},
         m_period_in_ms{period_in_ms}
   {
      error::verify(m_period_in_ms > 0);
   }

   void start()
   {
      m_bmp390.start();
   }

   void stop()
   {
      m_bmp390.stop();
   }

   void run_once()
   {
      m_bmp390.execute();
   }

   std::size_t get_period_ms() const
   {
      return m_period_in_ms;
   }

private:
   Bmp390&           m_bmp390;
   const std::size_t m_period_in_ms;
};

}   // namespace aeromight_barometer

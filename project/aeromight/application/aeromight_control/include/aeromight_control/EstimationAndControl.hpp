#pragma once

#include <cstdint>

namespace aeromight_control
{

template <typename Estimation, typename Control>
class EstimationAndControl
{
public:
   explicit EstimationAndControl(Estimation& estimation, Control& control, const std::size_t period_in_ms)
       : m_estimation{estimation},
         m_control{control},
         m_period_in_ms{period_in_ms}
   {
   }

   void start()
   {
      m_estimation.start();
      m_control.start();
   }

   void stop()
   {
      m_estimation.stop();
      m_control.stop();
   }

   void run_once()
   {
      m_estimation.execute();
      m_control.execute();
   }

   std::size_t get_period_ms() const
   {
      return m_period_in_ms;
   }

private:
   Estimation&       m_estimation;
   Control&          m_control;
   const std::size_t m_period_in_ms;
};

}   // namespace aeromight_control

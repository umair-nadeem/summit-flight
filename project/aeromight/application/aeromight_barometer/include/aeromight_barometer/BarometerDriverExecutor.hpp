#pragma once

#include "aeromight_boundaries/BarometerTaskEvents.hpp"
#include "error/error_handler.hpp"
#include "interfaces/peripherals/ISensorDriver.hpp"
#include "interfaces/rtos/INotificationWaiter.hpp"

namespace aeromight_barometer
{

template <interfaces::peripherals::ISensorDriver                                                  Bmp390,
          interfaces::rtos::INotificationWaiter<aeromight_boundaries::BarometerNotificationFlags> NotificationWaiter>
class BarometerDriverExecutor
{
public:
   explicit BarometerDriverExecutor(Bmp390&             bmp390,
                                    NotificationWaiter& notify_waiter,
                                    const std::size_t   period_in_ms,
                                    const std::size_t   notification_wait_period_in_ms)
       : m_bmp390{bmp390},
         m_notify_waiter{notify_waiter},
         m_period_in_ms{period_in_ms},
         m_notification_wait_period_in_ms{notification_wait_period_in_ms},
         m_rx_complete_notification{static_cast<uint8_t>(aeromight_boundaries::BarometerTaskEvents::rx_complete)}
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

      const auto flags = m_notify_waiter.wait(m_notification_wait_period_in_ms);
      if (flags.has_value())
      {
         // send rx complete event
         if (flags.value().test(m_rx_complete_notification))
         {
            m_bmp390.notify_receive_complete();
         }
      }
   }

   std::size_t get_period_ms() const
   {
      return m_period_in_ms;
   }

private:
   Bmp390&             m_bmp390;
   NotificationWaiter& m_notify_waiter;
   const std::size_t   m_period_in_ms;
   const std::size_t   m_notification_wait_period_in_ms;
   const uint8_t       m_rx_complete_notification;
};

}   // namespace aeromight_barometer

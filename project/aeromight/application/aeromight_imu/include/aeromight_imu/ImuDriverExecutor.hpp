#pragma once

#include <cstddef>

#include "aeromight_boundaries/ImuTaskEvents.hpp"
#include "error/error_handler.hpp"
#include "interfaces/pcb_component/ILed.hpp"
#include "interfaces/peripherals/ISensorDriver.hpp"
#include "interfaces/rtos/INotificationWaiter.hpp"
#include "sys_time/ClockSource.hpp"

namespace aeromight_imu
{

template <interfaces::peripherals::ISensorDriver                                            Mpu6500,
          interfaces::rtos::INotificationWaiter<aeromight_boundaries::ImuNotificationFlags> NotificationWaiter,
          interfaces::pcb_component::ILed                                                   Led,
          typename Logger>
class ImuDriverExecutor
{
public:
   explicit ImuDriverExecutor(Mpu6500& mpu6500, NotificationWaiter& notify_waiter, Led& led, Logger& logger, const uint32_t period_in_ms)
       : m_mpu6500{mpu6500},
         m_notify_waiter{notify_waiter},
         m_led{led},
         m_logger{logger},
         m_period_in_ms{period_in_ms},
         m_tick_notification{static_cast<uint8_t>(aeromight_boundaries::ImuTaskEvents::tick)},
         m_rx_complete_notification{static_cast<uint8_t>(aeromight_boundaries::ImuTaskEvents::rx_complete)}
   {
      error::verify(m_period_in_ms > 0);
      m_logger.enable();
   }

   void start()
   {
      m_mpu6500.start();
      m_logger.print("starting imu driver");
   }

   void stop()
   {
      m_mpu6500.stop();
      m_logger.print("stopping imu driver");
   }

   void run_once()
   {
      const auto flags = m_notify_waiter.wait(m_period_in_ms);
      if (flags.has_value())
      {
         // send rx complete event
         if (flags.value().test(m_rx_complete_notification))
         {
            m_mpu6500.notify_receive_complete();
         }

         // send tick event
         if (flags.value().test(m_tick_notification))
         {
            m_mpu6500.execute();
            blink_led();
         }
      }
   }

   uint32_t get_period_ms() const
   {
      return m_period_in_ms;
   }

private:
   void blink_led()
   {
      m_led_state_duration_counter += m_period_in_ms;
      if (m_led_state_duration_counter >= led_state_duration)
      {
         if (m_led_on)
         {
            m_led.turn_off();
            m_led_on = false;
         }
         else
         {
            m_led.turn_on();
            m_led_on = true;
         }

         m_led_state_duration_counter = 0;
      }
   }

   static constexpr uint32_t led_state_duration = 1000u;

   Mpu6500&            m_mpu6500;
   NotificationWaiter& m_notify_waiter;
   Led&                m_led;
   Logger&             m_logger;
   const uint32_t      m_period_in_ms;
   const uint8_t       m_tick_notification;
   const uint8_t       m_rx_complete_notification;
   std::size_t         m_led_state_duration_counter{0u};
   bool                m_led_on{false};
};

}   // namespace aeromight_imu

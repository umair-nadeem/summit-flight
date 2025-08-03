#pragma once

#include <cstddef>

#include "error/error_handler.h"

namespace aeromight_sensors
{

template <typename Led, typename Logger>
class SensorAcquisition
{
public:
   explicit SensorAcquisition(Led& led, Logger& logger, std::span<std::byte> tx_buffer, const std::size_t period_in_ms)
       : m_led{led},
         m_logger{logger},
         m_tx_buffer{tx_buffer},
         m_period_in_ms{period_in_ms}
   {
      error::verify(m_period_in_ms > 0);
   }

   void run_once()
   {
      blink_led();

      log_data();
   }

   std::size_t get_period_ms() const
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
            m_led.set_low();
            m_led_on = false;
         }
         else
         {
            m_led.set_high();
            m_led_on = true;
         }

         m_led_state_duration_counter = 0;
      }
   }

   void log_data()
   {
      m_logging_counter += m_period_in_ms;

      if (m_logging_counter >= 2 * led_state_duration)
      {
         m_tx_buffer[0] = std::byte{'1'};
         m_tx_buffer[1] = std::byte{'2'};
         m_tx_buffer[2] = std::byte{'3'};
         m_tx_buffer[3] = std::byte{'4'};
         m_tx_buffer[4] = std::byte{'5'};
         m_logger.send(5u);

         m_logging_counter = 0;
      }
   }

   static constexpr std::size_t led_state_duration = 2000u;

   Led&                 m_led;
   Logger&              m_logger;
   std::span<std::byte> m_tx_buffer;
   const std::size_t    m_period_in_ms;
   std::size_t          m_led_state_duration_counter{0u};
   std::size_t          m_logging_counter{0u};
   bool                 m_led_on{false};
};

}   // namespace aeromight_sensors

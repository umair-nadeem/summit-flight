#pragma once

#include <cstddef>

#include "error/error_handler.h"

namespace aeromight_sensors
{

template <typename Led, typename Logger>
class SensorAcquisition
{
public:
   explicit SensorAcquisition(Led& led, Logger& logger, std::span<std::byte> tx_buffer, const size_t period_in_ms)
       : m_led{led},
         m_logger{logger},
         m_tx_buffer{tx_buffer},
         m_period_in_ms{period_in_ms}
   {
      error::verify(m_period_in_ms > 0);
   }

   void run_once()
   {
      if (m_led_on)
      {
         m_on_time_counter += m_period_in_ms;

         if (m_on_time_counter >= m_target_on_time)
         {
            m_led.set_low();
            m_led_on           = false;
            m_on_time_counter  = 0u;
            m_off_time_counter = 0u;

            // adjust target_on_time for next time
            m_target_on_time += on_time_increment_delta;

            if (m_target_on_time > max_on_time)
            {
               m_target_on_time = min_on_time;
            }
         }
      }
      else
      {
         m_off_time_counter += m_period_in_ms;

         if (m_off_time_counter >= fixed_off_time)
         {
            m_led.set_high();
            m_led_on           = true;
            m_on_time_counter  = 0u;
            m_off_time_counter = 0u;
         }
      }

      log_data();
   }

   size_t get_period_ms() const
   {
      return m_period_in_ms;
   }

private:
   void log_data()
   {
      m_logging_counter += m_period_in_ms;

      if (m_logging_counter >= max_on_time)
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

   static constexpr size_t on_time_increment_delta = 500u;
   static constexpr size_t min_on_time             = 500u;
   static constexpr size_t max_on_time             = 4000u;
   static constexpr size_t fixed_off_time          = 1000u;

   Led&                 m_led;
   Logger&              m_logger;
   std::span<std::byte> m_tx_buffer;
   const size_t         m_period_in_ms;
   size_t               m_target_on_time{min_on_time};
   size_t               m_on_time_counter{0u};
   size_t               m_off_time_counter{0u};
   size_t               m_logging_counter{0u};
   bool                 m_led_on{false};
};

}   // namespace aeromight_sensors

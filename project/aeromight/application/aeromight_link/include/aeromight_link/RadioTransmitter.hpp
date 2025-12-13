#pragma once

#include <span>

#include "crsf/Crsf.hpp"
#include "error/error_handler.hpp"
#include "interfaces/IClockSource.hpp"
#include "interfaces/hw/IUartTransmitter.hpp"

namespace aeromight_link
{

template <interfaces::hw::IUartTransmitter UartTransmitter, typename Crsf, interfaces::IClockSource ClockSource>
class RadioTransmitter
{
public:
   explicit RadioTransmitter(UartTransmitter& uart_transmitter,
                             const uint32_t   battery_status_transmission_period_in_ms)
       : m_uart_transmitter{uart_transmitter},
         m_battery_status_transmission_period_in_ms{battery_status_transmission_period_in_ms}

   {
      get_battery_status();
   }

   void execute()
   {
      const uint32_t clock_ms = ClockSource::now_ms();
      if (clock_ms - m_last_battery_transmission_time > m_battery_status_transmission_period_in_ms)
      {
         std::span<std::byte> tx_buffer = m_uart_transmitter.get_buffer();

         // serialize
         const uint32_t bytes_written = Crsf::serialize_battery_telemetry(m_crsf_battery,
                                                                          std::span{reinterpret_cast<uint8_t*>(tx_buffer.data()), tx_buffer.size()});

         m_uart_transmitter.send_blocking(bytes_written);

         // update transmission timestamp
         m_last_battery_transmission_time = clock_ms;
      }
   }

private:
   void get_battery_status()
   {
      m_crsf_battery.voltage_v         = 5u;
      m_crsf_battery.current_a         = 10u;
      m_crsf_battery.capacity_used_mah = 1024u;
      m_crsf_battery.remaining_pct     = 50u;
   }

   UartTransmitter&  m_uart_transmitter;
   const uint32_t    m_battery_status_transmission_period_in_ms;
   crsf::CrsfBattery m_crsf_battery{};
   uint32_t          m_last_battery_transmission_time{0};
};

}   // namespace aeromight_link

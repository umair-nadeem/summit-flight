#pragma once

#include <span>

#include "boundaries/SharedData.hpp"
#include "crsf/Crsf.hpp"
#include "interfaces/IClockSource.hpp"
#include "interfaces/hw/IUartTransmitter.hpp"
#include "power/battery/BatteryStatus.hpp"

namespace aeromight_rc
{

template <interfaces::hw::IUartTransmitter UartTransmitter, typename Crsf, interfaces::IClockSource ClockSource>
class RadioTransmitter
{
   using BatteryStatusSubscriber = boundaries::SharedData<power::battery::BatteryStatus>;

public:
   explicit RadioTransmitter(UartTransmitter&               uart_transmitter,
                             const BatteryStatusSubscriber& battery_status_subscriber,
                             const uint32_t                 telemetry_transmission_period_ms)
       : m_uart_transmitter{uart_transmitter},
         m_battery_status_subscriber{battery_status_subscriber},
         m_telemetry_transmission_period_ms{telemetry_transmission_period_ms}

   {
   }

   void execute()
   {
      const uint32_t clock_ms = ClockSource::now_ms();

      if ((clock_ms - m_telemetry_transmission_time) >= m_telemetry_transmission_period_ms)
      {
         send_battery_telemetry();

         // update transmission timestamp
         m_telemetry_transmission_time = clock_ms;
      }
   }

private:
   void send_battery_telemetry()
   {
      const auto status = m_battery_status_subscriber.get_latest().data;

      crsf::CrsfBattery crsf_battery{};

      crsf_battery.voltage_10uv      = static_cast<int16_t>(status.voltage_mv / 100);   // 10 uV units
      crsf_battery.remaining_pct     = status.remaining_pct;
      crsf_battery.current_10ua      = 0;                                               // n/a
      crsf_battery.capacity_used_mah = 0;                                               // n/a

      // send telemetry
      std::span<std::byte> tx_buffer = m_uart_transmitter.get_buffer();

      // serialize
      const uint32_t bytes_written = Crsf::serialize_battery_telemetry(crsf_battery,
                                                                       std::span{reinterpret_cast<uint8_t*>(tx_buffer.data()), tx_buffer.size()});

      m_uart_transmitter.send_blocking(bytes_written);
   }

   UartTransmitter&               m_uart_transmitter;
   const BatteryStatusSubscriber& m_battery_status_subscriber;
   const uint32_t                 m_telemetry_transmission_period_ms;
   uint32_t                       m_telemetry_transmission_time{0};
};

}   // namespace aeromight_rc

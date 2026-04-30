#include "aeromight_rc/RadioTransmitter.hpp"

#include <gmock/gmock.h>

#include "mocks/common/Crsf.hpp"
#include "mocks/hw/UartTransmitter.hpp"
#include "mocks/sys_time/ClockSource.hpp"

class RadioTransmitterTest : public ::testing::Test
{
public:
   RadioTransmitterTest()
   {
      mocks::common::Crsf::reset();
      mocks::sys_time::ClockSource::reset();
   }

protected:
   static constexpr uint32_t battery_status_transmission_period_in_ms = 5u;

   std::array<std::byte, crsf::max_buffer_size>          uart_tx_buffer{};
   mocks::hw::UartTransmitter                            uart_transmitter_mock{};
   mocks::sys_time::ClockSource                          sys_clock{};
   boundaries::SharedData<power::battery::BatteryStatus> battery_status{};

   aeromight_rc::RadioTransmitter<decltype(uart_transmitter_mock),
                                  mocks::common::Crsf,
                                  mocks::sys_time::ClockSource>
       radio_transmitter{uart_transmitter_mock,
                         battery_status,
                         battery_status_transmission_period_in_ms};
};

TEST_F(RadioTransmitterTest, check_frequency_of_transmission)
{
   sys_clock.m_sec = battery_status_transmission_period_in_ms - 1u;

   EXPECT_CALL(uart_transmitter_mock, send_blocking).Times(0);
   radio_transmitter.execute();
}

TEST_F(RadioTransmitterTest, check_battery_telemetry)
{
   mocks::common::Crsf::telemetry_bytes_written = 10u;

   sys_clock.m_sec = battery_status_transmission_period_in_ms + 1u;

   EXPECT_CALL(uart_transmitter_mock, get_buffer()).WillOnce(testing::Return(std::span{uart_tx_buffer.data(), uart_tx_buffer.size()}));
   EXPECT_CALL(uart_transmitter_mock, send_blocking(10));
   radio_transmitter.execute();

   EXPECT_EQ(mocks::common::Crsf::battery_telemetry_packet.voltage_10uv, 5u);
   EXPECT_EQ(mocks::common::Crsf::battery_telemetry_packet.current_10ua, 10u);
   EXPECT_EQ(mocks::common::Crsf::battery_telemetry_packet.capacity_used_mah, 1024u);
   EXPECT_EQ(mocks::common::Crsf::battery_telemetry_packet.remaining_pct, 50u);

   EXPECT_EQ(mocks::common::Crsf::telemetry_out_buffer.data(), reinterpret_cast<const uint8_t*>(uart_tx_buffer.data()));
   EXPECT_EQ(mocks::common::Crsf::telemetry_out_buffer.size(), uart_tx_buffer.size());
}

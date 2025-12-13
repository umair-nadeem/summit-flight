#include "aeromight_link/RadioReceiver.hpp"

#include <cstdint>

#include <gmock/gmock.h>

#include "mocks/common/ClockSource.hpp"
#include "mocks/common/Crsf.hpp"
#include "mocks/common/Logger.hpp"
#include "mocks/rtos/QueueReceiver.hpp"
#include "mocks/rtos/QueueSender.hpp"

class RadioReceiverTest : public ::testing::Test
{
public:
   RadioReceiverTest()
   {
      mocks::common::Crsf::reset();
      mocks::common::ClockSource::reset();
   }

protected:
   static constexpr float   rc_channel_deadband           = 0.1f;
   static constexpr uint8_t good_uplink_quality_threshold = 50u;

   static constexpr uint16_t rc_channel_raw_deadband(uint16_t value) noexcept
   {
      return static_cast<uint16_t>(value * rc_channel_deadband);
   }

   mocks::rtos::QueueReceiver<boundaries::BufferWithOwnershipIndex<std::byte>> queue_receiver_mock{};
   mocks::rtos::QueueSender<std::size_t>                                       queue_sender_mock{};
   mocks::common::ClockSource                                                  sys_clock{};
   boundaries::SharedData<aeromight_boundaries::FlightSetpoints>               flight_setpoints_storage{};
   boundaries::SharedData<aeromight_boundaries::RadioLinkStats>                radio_link_actuals_storage{};
   mocks::common::Logger                                                       logger_mock{"radioReceiver"};

   aeromight_link::RadioReceiver<decltype(queue_receiver_mock),
                                 decltype(queue_sender_mock),
                                 mocks::common::Crsf,
                                 mocks::common::ClockSource,
                                 decltype(logger_mock)>
       radio_receiver{queue_receiver_mock,
                      queue_sender_mock,
                      flight_setpoints_storage,
                      radio_link_actuals_storage,
                      logger_mock,
                      rc_channel_deadband,
                      good_uplink_quality_threshold};
};

TEST_F(RadioReceiverTest, no_queue_item_is_received)
{
   EXPECT_CALL(queue_sender_mock, send_blocking).Times(0);

   radio_receiver.execute();
}

TEST_F(RadioReceiverTest, check_that_free_buffer_index_is_sent_over_buffer_pool_queue)
{
   for (std::size_t i = 0; i < 11u; i++)
   {
      boundaries::BufferWithOwnershipIndex<std::byte> message{.index = i};

      EXPECT_CALL(queue_receiver_mock, receive_if_available()).WillOnce(testing::Return(message));
      EXPECT_CALL(queue_sender_mock, send_blocking(i));

      radio_receiver.execute();
   }
}

TEST_F(RadioReceiverTest, crsf_parsing_failed)
{
   boundaries::BufferWithOwnershipIndex<std::byte> message{};
   crsf::CrsfLinkStatistics                        link_stats{};
   crsf::CrsfPacket                                packet{crsf::FrameType::link_statistics, link_stats};   // link stats

   mocks::common::Crsf::parse_result = false;                                                              // parsing result
   mocks::common::Crsf::crsf_packet  = packet;                                                             // update crsf packet that will be returned

   EXPECT_CALL(queue_receiver_mock, receive_if_available()).WillOnce(testing::Return(message));
   EXPECT_CALL(queue_sender_mock, send_blocking(0));

   sys_clock.m_sec = 1u;
   radio_receiver.execute();

   const auto radio_link_actuals = radio_link_actuals_storage.get_latest();

   EXPECT_EQ(radio_link_actuals.timestamp_ms, 0);
   EXPECT_EQ(radio_link_actuals.data.link_rssi_dbm, 0);
   EXPECT_EQ(radio_link_actuals.data.link_quality_pct, 0);
   EXPECT_EQ(radio_link_actuals.data.link_snr_db, 0);
   EXPECT_EQ(radio_link_actuals.data.tx_power_mw, 0);
   EXPECT_EQ(radio_link_actuals.data.link_status_ok, false);

   EXPECT_EQ(mocks::common::Crsf::buffer_to_parse.data(), reinterpret_cast<const uint8_t*>(message.buffer.data()));
}

TEST_F(RadioReceiverTest, check_rc_channels)
{
   boundaries::BufferWithOwnershipIndex<std::byte> message{};
   crsf::CrsfRcChannels                            rc_channels{};
   rc_channels.channels[aeromight_link::rc_channel_roll]        = crsf::rc_channel_value_min + rc_channel_raw_deadband(crsf::rc_channel_value_min);
   rc_channels.channels[aeromight_link::rc_channel_pitch]       = crsf::rc_channel_value_max - rc_channel_raw_deadband(crsf::rc_channel_value_max);
   rc_channels.channels[aeromight_link::rc_channel_throttle]    = crsf::rc_channel_value_min * 2u;
   rc_channels.channels[aeromight_link::rc_channel_yaw]         = (crsf::rc_channel_value_min + crsf::rc_channel_value_max) / 2u;
   rc_channels.channels[aeromight_link::rc_channel_arm_state]   = ((crsf::rc_channel_value_min + crsf::rc_channel_value_max) / 2u) + 1u;
   rc_channels.channels[aeromight_link::rc_channel_flight_mode] = ((crsf::rc_channel_value_min + crsf::rc_channel_value_max) / 2u);
   rc_channels.channels[aeromight_link::rc_channel_kill_switch] = ((crsf::rc_channel_value_min + crsf::rc_channel_value_max) / 2u) + crsf::rc_channel_value_min;

   crsf::CrsfPacket packet{crsf::FrameType::rc_channels_packed, rc_channels};   // rc channels

   mocks::common::Crsf::parse_result = true;                                    // parsing result
   mocks::common::Crsf::crsf_packet  = packet;                                  // update crsf packet that will be returned

   EXPECT_CALL(queue_receiver_mock, receive_if_available()).WillOnce(testing::Return(message));
   EXPECT_CALL(queue_sender_mock, send_blocking(0));

   sys_clock.m_sec = 1u;
   radio_receiver.execute();

   const auto flight_setpoints = flight_setpoints_storage.get_latest();

   EXPECT_NEAR(flight_setpoints.data.input.roll, -0.9795f, 0.01f);
   EXPECT_NEAR(flight_setpoints.data.input.pitch, 0.75459f, 0.01f);
   EXPECT_NEAR(flight_setpoints.data.input.throttle, 0.00549f, 0.01f);
   EXPECT_NEAR(flight_setpoints.data.input.yaw, 0.0f, 0.01f);
   EXPECT_EQ(flight_setpoints.timestamp_ms, sys_clock.m_sec);
   EXPECT_EQ(flight_setpoints.data.state, aeromight_boundaries::FlightArmedState::disarm);
   EXPECT_EQ(flight_setpoints.data.mode, aeromight_boundaries::FlightMode::altitude_hold);
   EXPECT_EQ(flight_setpoints.data.kill_switch_active, false);

   EXPECT_EQ(mocks::common::Crsf::buffer_to_parse.data(), reinterpret_cast<const uint8_t*>(message.buffer.data()));
}

TEST_F(RadioReceiverTest, check_link_stats)
{
   boundaries::BufferWithOwnershipIndex<std::byte> message{};
   crsf::CrsfLinkStatistics                        link_stats{};
   link_stats.uplink_rssi_1         = 1u;
   link_stats.uplink_rssi_2         = 2u;
   link_stats.uplink_link_quality   = 3u;
   link_stats.uplink_snr            = -4;
   link_stats.active_antenna        = 5u;
   link_stats.rf_profile            = 6u;
   link_stats.uplink_rf_power       = 7u;
   link_stats.downlink_rssi         = 8u;
   link_stats.downlink_link_quality = 9u;
   link_stats.downlink_snr          = -9;

   crsf::CrsfPacket packet{crsf::FrameType::link_statistics, link_stats};   // link stats

   mocks::common::Crsf::parse_result = true;                                // parsing result
   mocks::common::Crsf::crsf_packet  = packet;                              // update crsf packet that will be returned

   EXPECT_CALL(queue_receiver_mock, receive_if_available()).WillOnce(testing::Return(message));
   EXPECT_CALL(queue_sender_mock, send_blocking(0));

   sys_clock.m_sec = 1u;
   radio_receiver.execute();

   const auto radio_link_actuals = radio_link_actuals_storage.get_latest();

   EXPECT_EQ(radio_link_actuals.timestamp_ms, sys_clock.m_sec);
   EXPECT_EQ(radio_link_actuals.data.link_rssi_dbm, link_stats.uplink_rssi_1 * -1);
   EXPECT_EQ(radio_link_actuals.data.link_quality_pct, link_stats.uplink_link_quality);
   EXPECT_EQ(radio_link_actuals.data.link_snr_db, link_stats.uplink_snr);
   EXPECT_EQ(radio_link_actuals.data.tx_power_mw, crsf::get_uplink_rf_power_mw(link_stats.uplink_rf_power));
   EXPECT_EQ(radio_link_actuals.data.link_status_ok, (link_stats.uplink_link_quality > good_uplink_quality_threshold));

   EXPECT_EQ(mocks::common::Crsf::buffer_to_parse.data(), reinterpret_cast<const uint8_t*>(message.buffer.data()));
}

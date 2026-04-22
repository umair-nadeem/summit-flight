#include "aeromight_rc/RadioReceiver.hpp"

#include <cstdint>

#include <gmock/gmock.h>

#include "logging/Logger.hpp"
#include "mocks/common/ClockSource.hpp"
#include "mocks/common/Crsf.hpp"
#include "mocks/rtos/QueueReceiver.hpp"
#include "mocks/rtos/QueueSender.hpp"

class CrsfDecoderMock
{
public:
   MOCK_METHOD(rc::crsf::CrsfDecoderResult, decode_crsf, (std::span<const uint8_t>), ());
   MOCK_METHOD(rc::crsf::ChannelValues, get_rc_channels, (), ());
   MOCK_METHOD(rc::crsf::LinkStats, get_link_stats, (), ());
};

class RadioReceiverTest : public ::testing::Test
{
public:
   RadioReceiverTest()
   {
      mocks::common::Crsf::reset();
      mocks::common::ClockSource::reset();
   }

protected:
   static constexpr float rc_channel_deadband = 0.1f;

   static constexpr uint16_t rc_channel_raw_deadband(uint16_t value) noexcept
   {
      return static_cast<uint16_t>(value * rc_channel_deadband);
   }

   mocks::rtos::QueueReceiver<boundaries::BufferWithOwnershipIndex<std::byte>> queue_receiver_mock{};
   mocks::rtos::QueueSender<std::size_t>                                       queue_sender_mock{};
   CrsfDecoderMock                                                             crsf_decoder_mock{};
   mocks::common::ClockSource                                                  sys_clock{};
   boundaries::SharedData<rc::StickCommand>                                    stick_commands{};
   boundaries::SharedData<aeromight_boundaries::SystemControlSetpoints>        system_control_setpoints_storage{};
   boundaries::SharedData<rc::crsf::LinkStats>                                 link_stats_publisher{};
   logging::Logger                                                             logger_mock{"radioReceiver"};

   aeromight_rc::RadioReceiver<decltype(queue_receiver_mock),
                               decltype(queue_sender_mock),
                               decltype(crsf_decoder_mock),
                               mocks::common::ClockSource,
                               decltype(logger_mock)>
       radio_receiver{queue_receiver_mock,
                      queue_sender_mock,
                      crsf_decoder_mock,
                      stick_commands,
                      system_control_setpoints_storage,
                      link_stats_publisher,
                      logger_mock};
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

   const auto radio_link_actuals = link_stats_publisher.get_latest();

   EXPECT_EQ(radio_link_actuals.timestamp_ms, 0);
   EXPECT_EQ(radio_link_actuals.data.uplink_rssi_1_dbm, 0);
   EXPECT_EQ(radio_link_actuals.data.uplink_quality_pct, 0);
   EXPECT_EQ(radio_link_actuals.data.uplink_snr_db, 0);
   EXPECT_EQ(radio_link_actuals.data.tx_power_mw, 0);

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

   const auto radio_link_actuals = link_stats_publisher.get_latest();

   EXPECT_EQ(radio_link_actuals.timestamp_ms, sys_clock.m_sec);
   EXPECT_EQ(radio_link_actuals.data.uplink_rssi_1_dbm, link_stats.uplink_rssi_1 * -1);
   EXPECT_EQ(radio_link_actuals.data.uplink_quality_pct, link_stats.uplink_link_quality);
   EXPECT_EQ(radio_link_actuals.data.uplink_snr_db, link_stats.uplink_snr);
   EXPECT_EQ(radio_link_actuals.data.tx_power_mw, crsf::get_uplink_rf_power_mw(link_stats.uplink_rf_power));

   EXPECT_EQ(mocks::common::Crsf::buffer_to_parse.data(), reinterpret_cast<const uint8_t*>(message.buffer.data()));
}

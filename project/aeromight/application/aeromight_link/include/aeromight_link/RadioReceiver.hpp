#pragma once

#include "aeromight_boundaries/FlightManagerData.hpp"
#include "boundaries/BufferWithOwnershipIndex.hpp"
#include "channel_mappings.hpp"
#include "crsf/CrsfPacket.hpp"
#include "interfaces/IClockSource.hpp"

namespace aeromight_link
{

template <typename QueueReceiver, typename QueueSender, typename Crsf, interfaces::IClockSource ClockSource, typename Logger>
class RadioReceiver
{
   using SetpointsStorage = boundaries::SharedData<aeromight_boundaries::FlightManagerSetpoints>;
   using ActualsStorage   = boundaries::SharedData<aeromight_boundaries::FlightManagerActuals>;

public:
   explicit RadioReceiver(QueueReceiver&    radio_input_queue_receiver,
                          QueueSender&      free_index_queue_sender,
                          SetpointsStorage& setpoints_storage,
                          ActualsStorage&   actuals_storage,
                          Logger&           logger,
                          const float       rc_channel_deadband,
                          const uint8_t     good_uplink_quality_threshold)
       : m_radio_input_queue_receiver{radio_input_queue_receiver},
         m_free_index_queue_sender(free_index_queue_sender),
         m_setpoints_storage(setpoints_storage),
         m_actuals_storage(actuals_storage),
         m_logger(logger),
         m_rc_channel_deadband(rc_channel_deadband),
         m_good_uplink_quality_threshold(good_uplink_quality_threshold)
   {
      m_logger.enable();
   }

   void execute()
   {
      const auto queue_item = m_radio_input_queue_receiver.receive_if_available();
      if (queue_item.has_value())
      {
         const auto radio_input = queue_item.value();

         process_radio_input(std::span{reinterpret_cast<const uint8_t*>(radio_input.buffer.data()), radio_input.buffer.size()});

         m_free_index_queue_sender.send_blocking(radio_input.index);   // send index to queue to indicate buffer is free
      }
   }

private:
   void process_radio_input(std::span<const uint8_t> buffer)
   {
      const bool result = Crsf::parse_buffer(buffer, m_crsf_packet);

      if (!result)
      {
         m_logger.printf("received corrupted frame of len %u", buffer.size());
         return;
      }

      switch (m_crsf_packet.type)   // type
      {
         case crsf::FrameType::rc_channels_packed:
            publish_rc_channels(std::get<crsf::CrsfRcChannels>(m_crsf_packet.data));
            break;

         case crsf::FrameType::link_statistics:
            publish_link_stats(std::get<crsf::CrsfLinkStatistics>(m_crsf_packet.data));
            break;

         case crsf::FrameType::airspeed:
         case crsf::FrameType::attitude:
         case crsf::FrameType::battery_sensor:
         case crsf::FrameType::gps:
         case crsf::FrameType::heartbeat:
         case crsf::FrameType::link_statistics_rx:
         case crsf::FrameType::link_statistics_tx:
         default:
            error::stop_operation();
            break;
      }
   }

   void publish_rc_channels(const crsf::CrsfRcChannels& rc_data)
   {
      // update flight stick input
      m_setpoints.input.roll     = apply_deadband(normalize_channel(rc_data.channels[rc_channel_roll]), m_rc_channel_deadband);
      m_setpoints.input.pitch    = apply_deadband(normalize_channel(rc_data.channels[rc_channel_pitch]), m_rc_channel_deadband);
      m_setpoints.input.throttle = apply_deadband(normalize_throttle(rc_data.channels[rc_channel_throttle]), m_rc_channel_deadband);
      m_setpoints.input.yaw      = apply_deadband(normalize_channel(rc_data.channels[rc_channel_yaw]), m_rc_channel_deadband);

      // update armed state
      m_setpoints.state              = get_arm_state(normalize_channel(rc_data.channels[rc_channel_arm_state]));
      m_setpoints.mode               = get_flight_mode(normalize_channel(rc_data.channels[rc_channel_flight_mode]));
      m_setpoints.kill_switch_active = get_kill_switch_state(normalize_channel(rc_data.channels[rc_channel_kill_switch]));

      // publish to shared buffer
      m_setpoints_storage.update_latest(m_setpoints, ClockSource::now_ms());

      if (m_counter++ % 100 == 0)
      {
         m_logger.printf("arm=%u mode=%u kill=%u roll=%.2f pitch=%.2f throt=%.2f yaw=%.2f",
                         m_setpoints.state,
                         m_setpoints.mode,
                         m_setpoints.kill_switch_active,
                         m_setpoints.input.roll,
                         m_setpoints.input.pitch,
                         m_setpoints.input.throttle,
                         m_setpoints.input.yaw);
      }
   }

   void publish_link_stats(const crsf::CrsfLinkStatistics& stats)
   {
      // update stats
      m_actuals.link_rssi_dbm    = static_cast<int8_t>(stats.uplink_rssi_1) * -1;
      m_actuals.link_quality_pct = stats.uplink_link_quality;
      m_actuals.link_snr_db      = stats.uplink_snr;
      m_actuals.tx_power_mw      = crsf::get_uplink_rf_power_mw(stats.uplink_rf_power);
      m_actuals.link_status_ok   = (stats.uplink_link_quality > m_good_uplink_quality_threshold);

      // publish to shared buffer
      m_actuals_storage.update_latest(m_actuals, ClockSource::now_ms());

      if (m_counter++ % 250 == 0)
      {
         m_logger.printf("rssi=%d lq=%u snr=%d tx_pow=%u ls=%u",
                         m_actuals.link_rssi_dbm,
                         m_actuals.link_quality_pct,
                         m_actuals.link_snr_db,
                         m_actuals.tx_power_mw,
                         m_actuals.link_status_ok);
      }
   }

   static constexpr float normalize_channel(const uint16_t raw) noexcept
   {
      constexpr float center = (crsf::rc_channel_value_min + crsf::rc_channel_value_max) / 2.0f;   // 991.5

      return static_cast<float>(raw - center) / ((crsf::rc_channel_value_max - crsf::rc_channel_value_min) / 2.0f);
   }

   static constexpr float normalize_throttle(const uint16_t raw) noexcept
   {
      return static_cast<float>(raw - crsf::rc_channel_value_min) / (crsf::rc_channel_value_max - crsf::rc_channel_value_min);
   }

   static constexpr float apply_deadband(const float x, const float deadband) noexcept
   {
      if (fabs(x) < deadband)
      {
         return 0.0f;
      };

      return ((x > 0 ? (x - deadband) : (x + deadband)) / (1.0f - deadband));
   }

   static constexpr aeromight_boundaries::FlightArmedState get_arm_state(const float switch_value) noexcept
   {
      return (switch_value > rc_channel_switch_discrete_threshold) ? aeromight_boundaries::FlightArmedState::arm : aeromight_boundaries::FlightArmedState::disarm;
   }

   static constexpr aeromight_boundaries::FlightMode get_flight_mode(const float switch_value) noexcept
   {
      if (switch_value <= -rc_channel_switch_discrete_threshold)
      {
         return aeromight_boundaries::FlightMode::stabilized_manual;
      }
      else if ((-rc_channel_switch_discrete_threshold < switch_value) && (switch_value <= rc_channel_switch_discrete_threshold))
      {
         return aeromight_boundaries::FlightMode::altitude_hold;
      }
      else if (rc_channel_switch_discrete_threshold < switch_value)
      {
         return aeromight_boundaries::FlightMode::auto_land;
      }

      return aeromight_boundaries::FlightMode::none;
   }

   static constexpr bool get_kill_switch_state(const float switch_value) noexcept
   {
      return (switch_value > rc_channel_switch_discrete_threshold) ? true : false;
   }

   static constexpr float rc_channel_switch_discrete_threshold = 0.5f;

   QueueReceiver&                               m_radio_input_queue_receiver;
   QueueSender&                                 m_free_index_queue_sender;
   SetpointsStorage&                            m_setpoints_storage;
   ActualsStorage&                              m_actuals_storage;
   Logger&                                      m_logger;
   const float                                  m_rc_channel_deadband;
   const uint8_t                                m_good_uplink_quality_threshold;
   aeromight_boundaries::FlightManagerSetpoints m_setpoints{};
   aeromight_boundaries::FlightManagerActuals   m_actuals{};
   crsf::CrsfPacket                             m_crsf_packet{};
   std::size_t                                  m_counter{};
};

}   // namespace aeromight_link

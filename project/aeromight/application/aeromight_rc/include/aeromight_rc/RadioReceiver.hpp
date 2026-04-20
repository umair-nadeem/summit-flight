#pragma once

#include <cmath>

#include "aeromight_boundaries/SystemControlSetpoints.hpp"
#include "boundaries/BufferWithOwnershipIndex.hpp"
#include "boundaries/SharedData.hpp"
#include "channel_mappings.hpp"
#include "control/attitude/StickCommand.hpp"
#include "crsf/CrsfPacket.hpp"
#include "error/error_handler.hpp"
#include "interfaces/IClockSource.hpp"
#include "rc/crsf/Channel.hpp"
#include "rc/crsf/CrsfDecoderResult.hpp"
#include "rc/crsf/LinkStats.hpp"

namespace aeromight_rc
{

template <typename QueueReceiver, typename QueueSender, typename CrsfDecoder, interfaces::IClockSource ClockSource, typename Logger>
class RadioReceiver
{

   using StickCommandPublisher           = boundaries::SharedData<control::attitude::StickCommand>;
   using SystemControlSetpointsPublisher = boundaries::SharedData<aeromight_boundaries::SystemControlSetpoints>;
   using LinkStatsPublisher              = boundaries::SharedData<rc::crsf::LinkStats>;

public:
   explicit RadioReceiver(QueueReceiver&                   radio_input_queue_receiver,
                          QueueSender&                     free_index_queue_sender,
                          CrsfDecoder&                     crsf_decoder,
                          StickCommandPublisher&           stick_command_publisher,
                          SystemControlSetpointsPublisher& system_control_setpoints_publisher,
                          LinkStatsPublisher&              link_stats_publisher,
                          Logger&                          logger)
       : m_radio_input_queue_receiver{radio_input_queue_receiver},
         m_free_index_queue_sender{free_index_queue_sender},
         m_crsf_decoder{crsf_decoder},
         m_stick_command_publisher{stick_command_publisher},
         m_system_control_setpoints_publisher{system_control_setpoints_publisher},
         m_link_stats_publisher{link_stats_publisher},
         m_logger{logger}
   {
      m_logger.enable();
   }

   void execute()
   {
      const auto queue_item = m_radio_input_queue_receiver.receive_if_available();
      if (queue_item.has_value())
      {
         const auto radio_input = queue_item.value();

         const auto result = m_crsf_decoder.decode_crsf(std::span{reinterpret_cast<const uint8_t*>(radio_input.buffer.data()), radio_input.buffer.size()});

         const auto clock_ms = ClockSource::now_ms();

         if (result == rc::crsf::CrsfDecoderResult::rc_channels_update)
         {
            const auto& rc_channels = m_crsf_decoder.get_rc_channels();

            // system control setpoints
            const auto arm_state = get_bistate_channel(rc_channels, rc_channel_arm_state);

            m_system_control_setpoints.arm = (arm_state == rc::crsf::Bistate::high);

            const auto imu_calibration = get_bistate_channel(rc_channels, rc_channel_imu_calibration);

            m_system_control_setpoints.imu_calibration = (imu_calibration == rc::crsf::Bistate::high);

            // rc channels
            m_stick_command.roll     = get_float_channel(rc_channels, rc_channel_roll);
            m_stick_command.pitch    = get_float_channel(rc_channels, rc_channel_pitch);
            m_stick_command.yaw      = get_float_channel(rc_channels, rc_channel_yaw);
            m_stick_command.throttle = get_float_channel(rc_channels, rc_channel_throttle);

            m_system_control_setpoints_publisher.update_latest(m_system_control_setpoints, clock_ms);
            m_stick_command_publisher.update_latest(m_stick_command, clock_ms);
         }
         else if (result == rc::crsf::CrsfDecoderResult::link_stats_update)
         {
            const auto& link_stats = m_crsf_decoder.get_link_stats();
            m_link_stats_publisher.update_latest(link_stats, clock_ms);
         }
         else
         {
            error::stop_operation();
         }

         m_free_index_queue_sender.send_blocking(radio_input.index);   // send index to queue to indicate buffer is free
      }
   }

private:
   QueueReceiver&                               m_radio_input_queue_receiver;
   QueueSender&                                 m_free_index_queue_sender;
   CrsfDecoder&                                 m_crsf_decoder;
   StickCommandPublisher&                       m_stick_command_publisher;
   SystemControlSetpointsPublisher&             m_system_control_setpoints_publisher;
   LinkStatsPublisher&                          m_link_stats_publisher;
   Logger&                                      m_logger;
   control::attitude::StickCommand              m_stick_command{};
   aeromight_boundaries::SystemControlSetpoints m_system_control_setpoints{};
   rc::crsf::LinkStats                          m_radio_link_actuals{};
};

}   // namespace aeromight_rc

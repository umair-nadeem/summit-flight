#include "RadioLinkTaskData.hpp"
#include "aeromight_boundaries/AeromightData.hpp"
#include "aeromight_link/RadioLink.hpp"
#include "aeromight_link/RadioReceiver.hpp"
#include "aeromight_link/RadioTransmitter.hpp"
#include "crsf/Crsf.hpp"
#include "hw/uart/uart.hpp"
#include "logging/LogClient.hpp"
#include "rtos/periodic_task.hpp"
#include "sys_time/ClockSource.hpp"
#include "task_params.hpp"

namespace logging
{

extern rtos::QueueSender<params::LogBuffer> logging_queue_sender;

}   // namespace logging

extern "C"
{

   void radio_link_task(void* const params)
   {
      error::verify(params != nullptr);
      auto* data = static_cast<controller::RadioLinkTaskData*>(params);

      using LogClient = logging::LogClient<decltype(logging::logging_queue_sender)>;

      constexpr float    rc_channel_deadband                      = 0.1f;
      constexpr uint8_t  good_uplink_quality_threshold            = 50u;
      constexpr uint32_t battery_status_transmission_period_in_ms = 5000u;

      LogClient logger_radio_link{logging::logging_queue_sender, "radioLink"};

      aeromight_link::RadioTransmitter<decltype(data->radio_link_uart.transmitter),
                                       crsf::Crsf,
                                       sys_time::ClockSource>
          radio_transmitter{data->radio_link_uart.transmitter,
                            battery_status_transmission_period_in_ms};

      aeromight_link::RadioReceiver<decltype(data->radio_link_uart.radio_input_receiver),
                                    decltype(data->radio_link_uart.radio_queue_buffer_index_sender),
                                    crsf::Crsf,
                                    sys_time::ClockSource,
                                    decltype(logger_radio_link)>
          radio_receiver{
              data->radio_link_uart.radio_input_receiver,
              data->radio_link_uart.radio_queue_buffer_index_sender,
              aeromight_boundaries::aeromight_data.flight_manager_setpoints,
              aeromight_boundaries::aeromight_data.flight_manager_actuals,
              logger_radio_link,
              rc_channel_deadband,
              good_uplink_quality_threshold};

      aeromight_link::RadioLink<decltype(radio_receiver),
                                decltype(radio_transmitter)>
          radio_link{radio_receiver,
                     radio_transmitter,
                     controller::task::radio_link_task_period_in_ms};

      rtos::run_periodic_task(radio_link);
   }

}   // extern "C"

extern "C"
{
   void USART2_IRQHandler()
   {
      auto& data = controller::radio_link_task_data;
      hw::uart::handle_uart_global_interrupt(data.radio_link_uart.config, data.radio_link_uart.isr_sem_giver);
   }

   // uart rx
   void DMA1_Stream5_IRQHandler()
   {
      auto& data = controller::radio_link_task_data;
      hw::uart::handle_uart_dma_rx_global_interrupt<decltype(data.radio_link_uart.radio_input_sender_from_isr),
                                                    decltype(data.radio_link_uart.radio_queue_buffer_index_receiver_from_isr),
                                                    crsf::max_buffer_size>(data.radio_link_uart.config,
                                                                           data.radio_link_uart.radio_input_sender_from_isr,
                                                                           data.radio_link_uart.radio_queue_buffer_index_receiver_from_isr,
                                                                           data.radio_link_uart.dma_rx_buffer,
                                                                           std::span{data.radio_link_uart.user_rx_buffer_pool});
   }
}

#pragma once

#include <array>

#include "hw/gpio/DigitalOutput.hpp"
#include "hw/uart/Transmitter.h"
#include "hw/uart/UartConfig.h"
#include "rtos/RtosTaskConfig.h"
#include "rtos/SemaphoreGiver.h"
#include "rtos/SemaphoreTaker.h"
#include "task_params.h"

extern "C"
{
   [[noreturn]] void sensor_acquisition_task(void* params);
}

namespace controller
{

struct SensorAcquisitionTaskData
{
   hw::gpio::DigitalOutput blue_led{GPIOC, LL_GPIO_PIN_13, true};

   // logging uart
   struct LoggingUart
   {
      hw::uart::UartConfig config{
          .uart_handle   = USART1,
          .dma_handle    = DMA2,
          .tx_dma_stream = LL_DMA_STREAM_7,
          .rx_dma_stream = LL_DMA_STREAM_2};

      std::array<uint8_t, 64u> dma_tx_buffer{};
      std::array<uint8_t, 1u>  dummy_dma_rx_buffer{};

      rtos::SemaphoreTaker  transmitter_sem_taker{};
      rtos::SemaphoreGiver  isr_sem_giver{};
      hw::uart::Transmitter transmitter{config, std::as_writable_bytes(std::span{dma_tx_buffer}),
                                        [&]()
                                        { transmitter_sem_taker.take(); }};

   } logging_uart{};
};

extern SensorAcquisitionTaskData sensor_acq_task_data;

}   // namespace controller

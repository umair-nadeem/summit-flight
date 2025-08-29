#pragma once

#include "hw/HwPin.hpp"
#include "hw/spi/SpiConfig.hpp"
#include "hw/spi/SpiWithDmaConfig.hpp"
#include "hw/timer/TimerConfig.hpp"

namespace controller
{

struct GlobalData
{
   struct Gpio
   {
      hw::HwPin blue_led{GPIOC, LL_GPIO_PIN_13};
   } gpios;

   struct Spi
   {
      // SPI1
      hw::spi::SpiWithDmaConfig spi1_config{
          .spi_handle    = SPI1,
          .dma_handle    = DMA2,
          .tx_dma_stream = LL_DMA_STREAM_3,
          .rx_dma_stream = LL_DMA_STREAM_0};

      hw::HwPin spi1_chip_select{GPIOA, LL_GPIO_PIN_4};

      // SPI2
      hw::spi::SpiConfig spi2_config{
          .spi_handle = SPI2};

      hw::HwPin spi2_chip_select{GPIOB, LL_GPIO_PIN_2};
   } spi;

   struct Timer
   {
      // TIM2
      hw::timer::TimerConfig tim2_config{.timer_handle = TIM2};

      // TIM3
      hw::timer::TimerConfig tim3_config{.timer_handle = TIM3};
   } timer;
};

extern GlobalData global_data;

}   // namespace controller

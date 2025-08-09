#pragma once

#include "hw/HwPin.hpp"
#include "hw/spi/SpiConfig.hpp"

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
      // spi 1
      hw::spi::SpiConfig spi1_config{
          .spi_handle    = SPI1,
          .dma_handle    = DMA2,
          .tx_dma_stream = LL_DMA_STREAM_3,
          .rx_dma_stream = LL_DMA_STREAM_0};

      hw::HwPin spi1_chip_select{GPIOA, LL_GPIO_PIN_4};
   } spi;
};

extern GlobalData global_data;

}   // namespace controller

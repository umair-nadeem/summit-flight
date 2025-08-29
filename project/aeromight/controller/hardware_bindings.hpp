#pragma once

#include "hw/HwPin.hpp"
#include "hw/i2c/I2cConfig.hpp"
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
   } spi;

   struct I2c
   {
      // I2C1
      hw::i2c::I2cConfig i2c1_config{
          .i2c_handle = I2C1};
   } i2c;

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

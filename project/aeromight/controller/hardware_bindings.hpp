#pragma once

#include "hw/HwPin.hpp"

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
      hw::HwPin chip_select_spi1{GPIOA, LL_GPIO_PIN_4};
   } spi;
};

extern GlobalData global_data;

}   // namespace controller

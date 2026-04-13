#pragma once

#include "stm32f4xx_ll_gpio.h"

namespace hw
{

struct HwPin
{
   GPIO_TypeDef* port;
   uint32_t      pin;
};

}   // namespace hw

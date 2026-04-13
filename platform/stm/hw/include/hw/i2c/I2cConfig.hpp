#pragma once

#include "stm32f4xx_ll_i2c.h"

namespace hw::i2c
{

struct I2cConfig
{
   I2C_TypeDef*   i2c_handle{};
   const uint32_t frequency_hz{};
};

}   // namespace hw::i2c

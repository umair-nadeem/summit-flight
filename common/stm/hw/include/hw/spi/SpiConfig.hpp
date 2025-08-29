#pragma once

#include "stm32f4xx_ll_spi.h"

namespace hw::spi
{

struct SpiConfig
{
   SPI_TypeDef* spi_handle;
};

}   // namespace hw::spi

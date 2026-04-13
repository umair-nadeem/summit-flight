#pragma once

#include "stm32f4xx_ll_adc.h"

namespace hw::adc
{

struct AdcConfig
{
   ADC_TypeDef* adc_handle;
};

}   // namespace hw::adc

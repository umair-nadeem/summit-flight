#pragma once

#include "AdcConfig.hpp"
#include "error/error_handler.hpp"

namespace hw::adc
{

class Adc
{
public:
   explicit Adc(AdcConfig& config)
       : m_config{config}

   {
   }

   void prepare_for_communication()
   {
      // enable adc
      while (LL_ADC_IsEnabled(m_config.adc_handle) == 0u)
      {
         LL_ADC_Enable(m_config.adc_handle);
      }
   }

   void execute()
   {
      switch (m_state)
      {
         case AdcState::start_conversion:

            LL_ADC_REG_StartConversionSWStart(m_config.adc_handle);
            m_state = AdcState::read_value;
            break;

         case AdcState::read_value:
            if (LL_ADC_IsActiveFlag_EOCS(m_config.adc_handle) == 1u)
            {
               m_raw = LL_ADC_REG_ReadConversionData12(m_config.adc_handle);

               LL_ADC_ClearFlag_EOCS(m_config.adc_handle);

               m_state = AdcState::start_conversion;
            }
            break;

         default:
            error::stop_operation();
            break;
      }
   }

   uint16_t get_raw_value() const
   {
      return m_raw;
   }

private:
   enum class AdcState
   {
      start_conversion,
      read_value
   };

   AdcConfig& m_config;
   AdcState   m_state{AdcState::start_conversion};
   uint16_t   m_raw{0};
};

}   // namespace hw::adc

#pragma once

#include "VoltageSenseConfig.hpp"

namespace aeromight_battery
{

template <typename Adc, typename PercentageConvertor>
class Battery
{
public:
   explicit Battery(Adc&                      adc,
                    PercentageConvertor&      percentage_convertor,
                    const VoltageSenseConfig& config,
                    const float               battery_voltage_calibration_constant)
       : m_adc{adc},
         m_percentage_convertor{percentage_convertor},
         m_battery_voltage_calibration_constant{battery_voltage_calibration_constant},
         m_voltage_sense_factor{(((config.r1_ohm + config.r2_ohm) / config.r2_ohm) * config.vref_v) / config.resolution}
   {
   }

   void execute()
   {
      m_adc.execute();

      process_adc_raw_value();
   }

   int32_t get_voltage_mv() const
   {
      return m_voltage_mv;
   }

   uint8_t get_remaining_percentage() const
   {
      return m_remaining_percentage;
   }

private:
   void process_adc_raw_value()
   {
      const auto raw = m_adc.get_raw_value();

      const float adc_v = raw * m_voltage_sense_factor;
      m_voltage_mv      = static_cast<int32_t>(adc_v * m_battery_voltage_calibration_constant * 1000.0f);   // voltage

      m_remaining_percentage = m_percentage_convertor.to_percentage(m_voltage_mv);                          // percentage
   }

   Adc&                 m_adc;
   PercentageConvertor& m_percentage_convertor;
   const float          m_battery_voltage_calibration_constant;
   const float          m_voltage_sense_factor;
   int32_t              m_voltage_mv{0};
   uint8_t              m_remaining_percentage{0};
};

}   // namespace aeromight_battery

#pragma once

#include "power/battery/BatteryStatus.hpp"
#include "power/battery/VoltageSenseConfig.hpp"

namespace power::battery
{

template <typename Adc, typename BatteryPercentageModel>
class Battery
{
public:
   explicit Battery(Adc&                      adc,
                    BatteryPercentageModel&   percentage_model,
                    const VoltageSenseConfig& config,
                    const float               battery_voltage_calibration_constant)
       : m_adc{adc},
         m_percentage_model{percentage_model},
         m_battery_voltage_calibration_constant{battery_voltage_calibration_constant},
         m_voltage_sense_factor{(((config.r1_ohm + config.r2_ohm) / config.r2_ohm) * config.vref_v) / static_cast<float>(config.resolution)}
   {
   }

   void execute()
   {
      m_adc.execute();

      process_adc_raw_value();
   }

   const BatteryStatus& get_status() const
   {
      return m_battery_status;
   }

private:
   void process_adc_raw_value()
   {
      const auto raw = m_adc.get_raw_value();

      const float adc_v = raw * m_voltage_sense_factor;

      m_battery_status.voltage_mv = static_cast<int32_t>(adc_v * m_battery_voltage_calibration_constant * 1000.0f);   // voltage

      m_battery_status.remaining_pct = m_percentage_model.to_percentage(m_battery_status.voltage_mv);                 // percentage
   }

   Adc&                    m_adc;
   BatteryPercentageModel& m_percentage_model;
   const float             m_battery_voltage_calibration_constant;
   const float             m_voltage_sense_factor;
   BatteryStatus           m_battery_status{};
};

}   // namespace power::battery

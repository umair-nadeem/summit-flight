#pragma once

#include "barometer_sensor/BarometerData.hpp"
#include "boundaries/SharedData.hpp"
#include "imu_sensor/ImuData.hpp"

namespace aeromight_control
{

template <typename Logger>
class Estimation
{
   using ImuData       = ::boundaries::SharedData<imu_sensor::ImuData>;
   using BarometerData = ::boundaries::SharedData<barometer_sensor::BarometerData>;

public:
   explicit Estimation(const ImuData& imu_data, const BarometerData& barometer_data, Logger& logger)
       : m_imu_data{imu_data},
         m_barometer_data{barometer_data},
         m_logger{logger}
   {
      m_logger.enable();
   }

   void start()
   {
      m_enabled = true;
      m_logger.print("started estimation");
   }

   void stop()
   {
      m_enabled = false;
      m_logger.print("stopped estimation");
   }

   void execute() const
   {
      if (m_enabled)
      {
      }
   }

private:
   const ImuData&       m_imu_data;
   const BarometerData& m_barometer_data;
   Logger&              m_logger;
   bool                 m_enabled{false};
};

}   // namespace aeromight_control

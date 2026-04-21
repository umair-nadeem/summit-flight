#pragma once

#include "Events.hpp"
#include "MainSM.hpp"
#include "Mpu6500StateHandler.hpp"

namespace mpu6500
{

template <typename SpiMaster, typename Logger>
class Mpu6500
{
public:
   explicit Mpu6500(SpiMaster&                    spi_master,
                    Logger&                       logger,
                    imu_sensor::RawImuSensorData& raw_sensor_data_out,
                    imu_sensor::ImuSensorStatus&  sensor_status_out,
                    const Mpu6500Params&          params)
       : m_state_handler{spi_master,
                         logger,
                         raw_sensor_data_out,
                         sensor_status_out,
                         params}
   {
   }

   void execute()
   {
      m_state_machine.process_event(EventTick{});
   }

   void start()
   {
      m_state_machine.process_event(EventStart{});
   }

   void stop()
   {
      m_state_machine.process_event(EventStop{});
   }

   void notify_receive_complete()
   {
      m_state_machine.process_event(EventReceiveDone{});
   }

   imu_sensor::RawImuSensorData get_raw_data() const
   {
      return m_state_handler.get_raw_data();
   }

   imu_sensor::ImuSensorStatus get_status() const
   {
      return m_state_handler.get_status();
   }

   imu_sensor::ImuSensorErrorBits get_error() const
   {
      return m_state_handler.get_error();
   }

   mpu6500::SensorState get_state() const
   {
      return m_state_handler.get_state();
   }

private:
   using StateHandler    = Mpu6500StateHandler<SpiMaster, Logger>;
   using StateMachineDef = MainStateMachine<StateHandler>;

   StateHandler                    m_state_handler;
   boost::sml::sm<StateMachineDef> m_state_machine{m_state_handler};
};

}   // namespace mpu6500

#pragma once

#include "Bmp390StateHandler.hpp"
#include "MainSM.hpp"

namespace bmp390
{

template <typename I2cDriver, typename Logger>
class Bmp390
{
public:
   explicit Bmp390(I2cDriver&                                i2c_driver,
                   Logger&                                   logger,
                   barometer_sensor::RawBarometerSensorData& raw_sensor_data_out,
                   barometer_sensor::BarometerSensorStatus&  sensor_status_out,
                   const uint8_t                             read_failures_limit,
                   const uint8_t                             max_recovery_attempts,
                   const uint32_t                            execution_period_ms,
                   const uint32_t                            receive_wait_timeout_ms)
       : m_state_handler{i2c_driver,
                         logger,
                         raw_sensor_data_out,
                         sensor_status_out,
                         read_failures_limit,
                         max_recovery_attempts,
                         execution_period_ms,
                         receive_wait_timeout_ms},
         m_logger{logger}
   {
      m_logger.enable();
   }

   void start()
   {
      m_state_machine.process_event(EventStart{});
   }

   void stop()
   {
      m_state_machine.process_event(EventStop{});
   }

   void execute()
   {
      m_state_machine.process_event(EventTick{});
   }

   void notify_receive_complete()
   {
      m_state_machine.process_event(EventReceiveDone{});
   }

   barometer_sensor::RawBarometerSensorData get_raw_data() const
   {
      return m_state_handler.get_raw_data();
   }

   barometer_sensor::BarometerSensorStatus get_status() const
   {
      return m_state_handler.get_status();
   }

   barometer_sensor::BarometerSensorErrorBits get_error() const
   {
      return m_state_handler.get_error();
   }

   bmp390::SensorState get_state() const
   {
      return m_state_handler.get_state();
   }

private:
   using StateHandler    = Bmp390StateHandler<I2cDriver, Logger>;
   using StateMachineDef = Bmp390MainStateMachine<StateHandler>;

   StateHandler                    m_state_handler;
   Logger&                         m_logger;
   boost::sml::sm<StateMachineDef> m_state_machine{m_state_handler};
};

}   // namespace bmp390

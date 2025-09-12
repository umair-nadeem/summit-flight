#pragma once

#include "Bmp390StateHandler.hpp"
#include "MainSM.hpp"

namespace bmp390
{

template <interfaces::IClockSource ClockSource, typename I2cDriver, typename Logger>
class Bmp390
{
   using BarometerData   = ::boundaries::SharedData<barometer_sensor::BarometerData>;
   using BarometerHealth = ::boundaries::SharedData<barometer_sensor::BarometerHealth>;

public:
   explicit Bmp390(BarometerData&    barometer_data_storage,
                   BarometerHealth&  barometer_health_storage,
                   I2cDriver&        i2c_driver,
                   Logger&           logger,
                   const uint8_t     read_failures_limit,
                   const uint8_t     max_recovery_attempts,
                   const std::size_t execution_period_ms,
                   const std::size_t receive_wait_timeout_ms)
       : m_state_handler{barometer_data_storage,
                         barometer_health_storage,
                         i2c_driver,
                         logger,
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
      m_logger.print("starting barometer driver");
   }

   void stop()
   {
      m_state_machine.process_event(EventStop{});
      m_logger.print("stopping barometer driver");
   }

   void execute()
   {
      m_state_machine.process_event(EventTick{});
   }

   void notify_receive_complete()
   {
      m_state_machine.process_event(EventReceiveDone{});
   }

   barometer_sensor::BarometerSensorState get_state() const
   {
      return m_state_handler.get_state();
   }

   barometer_sensor::ErrorBits get_error() const
   {
      return m_state_handler.get_error();
   }

private:
   using StateHandler    = Bmp390StateHandler<ClockSource, I2cDriver, Logger>;
   using StateMachineDef = Bmp390MainStateMachine<StateHandler>;

   StateHandler                    m_state_handler;
   Logger&                         m_logger;
   boost::sml::sm<StateMachineDef> m_state_machine{m_state_handler};
};

}   // namespace bmp390

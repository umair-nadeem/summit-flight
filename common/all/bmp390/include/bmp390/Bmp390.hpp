#pragma once

#include <boost/sml.hpp>

#include "Bmp390StateHandler.hpp"
#include "SetupSM.hpp"

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
                   const std::size_t execution_period_ms,
                   const std::size_t receive_wait_timeout_ms)
       : m_state_handler{barometer_data_storage,
                         barometer_health_storage,
                         i2c_driver,
                         logger,
                         read_failures_limit,
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

private:
   template <typename StateHandler>
   struct Bmp390StateMachine
   {
      // wrappers for setup substate machine
      template <typename H>
      struct InitSetupSM : SetupStateMachine<H>
      {
      };

      // composite state machines
      static constexpr auto s_init_setup = boost::sml::state<InitSetupSM<StateHandler>>;

      // leaf states
      static constexpr auto s_stopped                = boost::sml::state<class StateStopped>;
      static constexpr auto s_read_coefficients      = boost::sml::state<class StateReadCoefficients>;
      static constexpr auto s_read_coefficients_wait = boost::sml::state<class StateReadCoefficientsWait>;
      static constexpr auto s_measurement            = boost::sml::state<class StateMeasurement>;
      static constexpr auto s_data_read_wait         = boost::sml::state<class StateDataReadWait>;
      static constexpr auto s_data_verification      = boost::sml::state<class StateDataVerification>;
      static constexpr auto s_data_read_fail         = boost::sml::state<class StateDataReadFailure>;
      static constexpr auto s_failure                = boost::sml::state<class StateFailure>;

      auto operator()() const
      {
         using namespace boost::sml;

         // actions
         static constexpr auto reset_timer = [](StateHandler& state)
         { state.reset_timer(); };

         static constexpr auto tick_timer = [](StateHandler& state)
         { state.tick_timer(); };

         constexpr auto reset_read_failures = [](StateHandler& state)
         { state.reset_read_failures(); };

         constexpr auto count_read_failure = [](StateHandler& state)
         { state.count_read_failure(); };

         static constexpr auto read_coefficients = [](StateHandler& state)
         { state.read_coefficients(); };

         static constexpr auto read_data = [](StateHandler& state)
         { state.read_data(); };

         static constexpr auto store_coefficients = [](StateHandler& state)
         { state.store_coefficients(); };

         static constexpr auto process_error_register = [](StateHandler& state)
         { state.process_error_register(); };

         static constexpr auto convert_raw_data = [](StateHandler& state)
         { state.convert_raw_data(); };

         constexpr auto publish_data = [](StateHandler& state)
         { state.publish_data(); };

         constexpr auto publish_health = [](StateHandler& state)
         { state.publish_health(); };

         constexpr auto reset_data = [](StateHandler& state)
         { state.reset_data(); };

         static constexpr auto set_setup_state = [](StateHandler& state)
         { state.set_state(barometer_sensor::BarometerSensorState::setup); };

         static constexpr auto set_operational_state = [](StateHandler& state)
         { state.set_state(barometer_sensor::BarometerSensorState::operational); };

         static constexpr auto set_failure_state = [](StateHandler& state)
         { state.set_state(barometer_sensor::BarometerSensorState::failure); };

         static constexpr auto set_bus_error = [](StateHandler& state)
         { state.set_error(barometer_sensor::BarometerSensorError::bus_error); };

         static constexpr auto set_sensor_error = [](StateHandler& state)
         { state.set_error(barometer_sensor::BarometerSensorError::sensor_error); };

         static constexpr auto set_data_error = [](StateHandler& state)
         { state.set_error(barometer_sensor::BarometerSensorError::data_error); };

         // guards
         constexpr auto validation_successful = [](StateHandler& state)
         { return state.validation_successful(); };

         constexpr auto config_successful = [](StateHandler& state)
         { return state.config_successful(); };

         constexpr auto is_buffer_non_zero = [](const StateHandler& state)
         { return state.is_buffer_non_zero(); };

         constexpr auto sensor_error_reported = [](const StateHandler& state)
         { return state.sensor_error_reported(); };

         constexpr auto is_data_valid = [](const StateHandler& state)
         { return state.is_data_valid(); };

         static constexpr auto receive_wait_timeout = [](StateHandler& state)
         { return state.receive_wait_timeout(); };

         // events
         static constexpr auto e_start        = event<EventStart>;
         static constexpr auto e_tick         = event<EventTick>;
         static constexpr auto e_receive_done = event<EventReceiveDone>;

         return make_transition_table(
             // From State  | Event   | Guard                    | Action                                  | To State
             *s_stopped + e_start / set_setup_state = s_init_setup,

             // setup
             s_init_setup[!validation_successful]                     = s_failure,
             s_init_setup[!config_successful]                         = s_failure,
             s_init_setup[validation_successful && config_successful] = s_read_coefficients,

             // read coefficients
             s_read_coefficients + e_tick / (reset_timer, read_coefficients)                         = s_read_coefficients_wait,
             s_read_coefficients_wait + e_receive_done / (store_coefficients, set_operational_state) = s_measurement,
             s_read_coefficients_wait + e_tick[!receive_wait_timeout] / tick_timer,
             s_read_coefficients_wait + e_tick[receive_wait_timeout] / set_bus_error = s_failure,

             // operational
             s_measurement + e_tick / (reset_timer, read_data) = s_data_read_wait,

             s_data_read_wait + e_receive_done[is_buffer_non_zero] / (process_error_register, convert_raw_data) = s_data_verification,
             s_data_read_wait + e_receive_done[!is_buffer_non_zero] / (set_sensor_error, count_read_failure)    = s_data_read_fail,
             s_data_read_wait + e_tick[!receive_wait_timeout] / tick_timer,
             s_data_read_wait + e_tick[receive_wait_timeout] / (set_bus_error, count_read_failure) = s_data_read_fail,

             s_data_verification[sensor_error_reported] / set_sensor_error              = s_failure,
             s_data_verification[is_data_valid] / (reset_read_failures, publish_data)   = s_measurement,
             s_data_verification[!is_data_valid] / (set_data_error, count_read_failure) = s_data_read_fail,

             s_failure + boost::sml::on_entry<_> / (set_failure_state, publish_health, reset_data));
      }
   };

   using StateHandler    = Bmp390StateHandler<ClockSource, I2cDriver, Logger>;
   using StateMachineDef = Bmp390StateMachine<StateHandler>;

   StateHandler                    m_state_handler;
   Logger&                         m_logger;
   boost::sml::sm<StateMachineDef> m_state_machine{m_state_handler};
};

}   // namespace bmp390

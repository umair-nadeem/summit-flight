#pragma once

#include <boost/sml.hpp>

#include "Bmp390StateHandler.hpp"

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
                   const std::size_t execution_period_ms)
       : m_state_handler{barometer_data_storage,
                         barometer_health_storage,
                         i2c_driver,
                         logger,
                         read_failures_limit,
                         execution_period_ms},
         m_logger{logger}
   {
      m_logger.enable();
   }

   void start()
   {
      m_state_machine.process_event(typename StateMachineDef::EventStart{});
      m_logger.print("starting barometer driver");
   }

   void stop()
   {
      m_state_machine.process_event(typename StateMachineDef::EventStop{});
      m_logger.print("stopping barometer driver");
   }

   void execute()
   {
      m_state_machine.process_event(typename StateMachineDef::EventTick{});
   }

private:
   template <typename StateHandler>
   struct Bmp390StateMachine
   {
      // states
      static constexpr auto s_stopped    = boost::sml::state<class StateStopped>;
      static constexpr auto s_init_reset = boost::sml::state<class StateInitReset>;

      static constexpr auto s_init_read_id = boost::sml::state<class StateInitReadId>;
      static constexpr auto s_init_verify  = boost::sml::state<class StateInitVerifyId>;

      static constexpr auto s_init_config       = boost::sml::state<class StateInitConfig>;
      static constexpr auto s_init_config_check = boost::sml::state<class StateInitConfigCheck>;

      static constexpr auto s_measurement    = boost::sml::state<class StateMeasurement>;
      static constexpr auto s_data_read_wait = boost::sml::state<class StateDataReadWait>;
      static constexpr auto s_failure        = boost::sml::state<class StateFailure>;

      // events
      struct EventTick
      {
      };

      struct EventStart
      {
      };

      struct EventStop
      {
      };

      auto operator()() const
      {
         using namespace boost::sml;

         // actions
         static constexpr auto tick_timer = [](StateHandler& state)
         { state.tick_timer(); };

         static constexpr auto soft_reset = [](StateHandler& state)
         { state.soft_reset(); };

         static constexpr auto read_id = [](StateHandler& state)
         { state.read_id(); };

         static constexpr auto write_config = [](StateHandler& state)
         { state.write_config(); };

         static constexpr auto read_data = [](StateHandler& state)
         { state.read_data(); };

         static constexpr auto store_id = [](StateHandler& state)
         { state.store_id(); };

         static constexpr auto convert_raw_data = [](StateHandler& state)
         { state.convert_raw_data(); };

         static constexpr auto set_reset_state = [](StateHandler& state)
         { state.set_state(barometer_sensor::BarometerSensorState::reset); };

         static constexpr auto set_validation_state = [](StateHandler& state)
         { state.set_state(barometer_sensor::BarometerSensorState::validation); };

         static constexpr auto set_config_state = [](StateHandler& state)
         { state.set_state(barometer_sensor::BarometerSensorState::config); };

         static constexpr auto set_operational_state = [](StateHandler& state)
         { state.set_state(barometer_sensor::BarometerSensorState::operational); };

         static constexpr auto set_bus_error = [](StateHandler& state)
         { state.set_error(barometer_sensor::BarometerSensorError::bus_error); };

         static constexpr auto set_sensor_error = [](StateHandler& state)
         { state.set_error(barometer_sensor::BarometerSensorError::sensor_error); };

         // static constexpr auto set_data_error = [](StateHandler& state)
         // { state.set_error(barometer_sensor::BarometerSensorError::data_error); };

         // guards
         static constexpr auto bus_error = [](StateHandler& state)
         { return state.bus_error(); };

         constexpr auto id_matched = [](const StateHandler& state)
         { return state.id_matched(); };

         // events
         static constexpr auto e_start = event<EventStart>;
         static constexpr auto e_tick  = event<EventTick>;

         // clang-format off
         return make_transition_table(
             // From State  | Event   | Guard                    | Action                                  | To State
             *s_stopped         + e_start                     / set_reset_state                          = s_init_reset,

             s_init_reset       + e_tick                      / (soft_reset, set_validation_state)       = s_init_read_id,

             s_init_read_id     + e_tick                      / read_id                                  = s_init_verify,

             s_init_verify      + e_tick  [bus_error]    / set_bus_error                             = s_failure,
             s_init_verify      + e_tick  [!id_matched]  / (store_id, set_sensor_error)              = s_failure,
             s_init_verify      + e_tick  [id_matched]   / (store_id, set_config_state)              = s_init_config,

             s_init_config      + e_tick                 / write_config                             = s_init_config_check,

             s_init_config_check          [bus_error]    / set_bus_error                             = s_failure,
             s_init_config_check          [!bus_error]    / set_operational_state                    = s_measurement,

             s_measurement      + e_tick                 / read_data                        = s_data_read_wait,
             s_data_read_wait   + e_tick                 / convert_raw_data                 = s_measurement,

             s_failure          + e_tick                 / tick_timer
            );
         // clang-format on
      }
   };

   using StateHandler    = Bmp390StateHandler<ClockSource, I2cDriver, Logger>;
   using StateMachineDef = Bmp390StateMachine<StateHandler>;

   StateHandler                    m_state_handler;
   Logger&                         m_logger;
   boost::sml::sm<StateMachineDef> m_state_machine{m_state_handler};
};

}   // namespace bmp390

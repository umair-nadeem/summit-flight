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
      static constexpr auto s_stopped   = boost::sml::state<class StateStopped>;
      static constexpr auto s_read_id   = boost::sml::state<class StateReadId>;
      static constexpr auto s_verify_id = boost::sml::state<class StateVerifyId>;
      static constexpr auto s_config    = boost::sml::state<class StateConfig>;
      static constexpr auto s_failure   = boost::sml::state<class StateFailure>;

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

         static constexpr auto reset = [](StateHandler& state)
         { state.reset(); };

         static constexpr auto read_id = [](StateHandler& state)
         { state.read_id(); };

         static constexpr auto store_id = [](StateHandler& state)
         { state.store_id(); };

         static constexpr auto set_config_state = [](StateHandler& state)
         { state.set_config_state(); };

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
             *s_stopped     + e_start               / reset                          = s_read_id,

             s_read_id      + e_tick                / read_id                         = s_verify_id,

             s_verify_id              [bus_error]    / set_bus_error                = s_failure,
             s_verify_id              [!id_matched] / (store_id, set_sensor_error)       = s_failure,
             s_verify_id              [id_matched]   / (store_id, set_config_state)     = s_config,

             s_config        + e_tick                 / tick_timer,
             s_failure      + e_tick                 / tick_timer
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

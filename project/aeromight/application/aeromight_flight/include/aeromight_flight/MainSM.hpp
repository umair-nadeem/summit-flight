#pragma once

#include <boost/sml.hpp>

#include "FlightManagerState.hpp"

namespace aeromight_flight
{

template <typename StateHandler>
struct FlightManagerStateMachine
{

   // leaf states
   static constexpr auto s_init                    = boost::sml::state<class StateInit>;
   static constexpr auto s_init_checkpoint         = boost::sml::state<class StateInitCheckpoint>;
   static constexpr auto s_wait_sensors            = boost::sml::state<class StateWaitSensors>;
   static constexpr auto s_wait_sensors_checkpoint = boost::sml::state<class StateWaitSensorsCheckpoint>;
   static constexpr auto s_wait_control            = boost::sml::state<class StateWaitControl>;
   static constexpr auto s_wait_control_checkpoint = boost::sml::state<class StateWaitControlCheckpoint>;
   static constexpr auto s_disarmed                = boost::sml::state<class StateIdle>;
   static constexpr auto s_disarmed_checkpoint     = boost::sml::state<class StateIdleCheckpoint>;
   static constexpr auto s_arming                  = boost::sml::state<class StateArming>;
   static constexpr auto s_arming_checkpoint       = boost::sml::state<class StateArmingCheckpoint>;
   static constexpr auto s_armed                   = boost::sml::state<class StateArmed>;
   static constexpr auto s_armed_checkpoint        = boost::sml::state<class StateArmedCheckpoint>;
   static constexpr auto s_disarming               = boost::sml::state<class StateDisarming>;
   static constexpr auto s_disarming_checkpoint    = boost::sml::state<class StateDisarmingCheckpoint>;
   static constexpr auto s_manual                  = boost::sml::state<class StateManual>;
   static constexpr auto s_manual_checkpoint       = boost::sml::state<class StateManualCheckpoint>;
   static constexpr auto s_hover                   = boost::sml::state<class StateHover>;
   static constexpr auto s_hover_checkpoint        = boost::sml::state<class StateHoverCheckpoint>;
   static constexpr auto s_auto_land               = boost::sml::state<class StateAutoLand>;
   static constexpr auto s_auto_land_checkpoint    = boost::sml::state<class StateAutoLandCheckpoint>;
   static constexpr auto s_killed                  = boost::sml::state<class StateKilled>;
   static constexpr auto s_killed_checkpoint       = boost::sml::state<class StateKilledCheckpoint>;
   static constexpr auto s_to_fault                = boost::sml::state<class StateToFault>;
   static constexpr auto s_fault                   = boost::sml::state<class StateFault>;

   // events
   struct EventTick
   {
   };

   auto operator()() const
   {
      using namespace boost::sml;

      // actions
      constexpr auto set_reference_time = [](StateHandler& state)
      {
         state.set_reference_time();
      };

      constexpr auto get_time = [](StateHandler& state)
      {
         state.get_time();
      };

      constexpr auto read_health_summary = [](StateHandler& state)
      {
         state.read_health_summary();
      };

      constexpr auto read_radio_input = [](StateHandler& state)
      {
         state.read_radio_input();
      };

      constexpr auto start_control = [](StateHandler& state)
      {
         state.start_control();
      };

      constexpr auto start_motors = [](StateHandler& state)
      {
         state.start_motors();
      };

      constexpr auto stop_motors = [](StateHandler& state)
      {
         state.stop_motors();
      };

      constexpr auto publish_manual_setpoint = [](StateHandler& state)
      {
         state.publish_manual_setpoint();
      };

      constexpr auto publish_hover_setpoint = [](StateHandler& state)
      {
         state.publish_hover_setpoint();
      };

      constexpr auto publish_auto_land_setpoint = [](StateHandler& state)
      {
         state.publish_auto_land_setpoint();
      };

      constexpr auto set_wait_sensors_state = [](StateHandler& state)
      {
         state.set_state(FlightManagerState::wait_sensors);
      };

      constexpr auto set_wait_control_state = [](StateHandler& state)
      {
         state.set_state(FlightManagerState::wait_control);
      };

      constexpr auto set_disarmed_state = [](StateHandler& state)
      {
         state.set_state(FlightManagerState::disarmed);
      };

      constexpr auto set_armed_state = [](StateHandler& state)
      {
         state.set_state(FlightManagerState::armed);
      };

      constexpr auto set_manual_state = [](StateHandler& state)
      {
         state.set_state(FlightManagerState::manual_mode);
      };

      constexpr auto set_hover_state = [](StateHandler& state)
      {
         state.set_state(FlightManagerState::hover_mode);
      };

      constexpr auto set_auto_land_state = [](StateHandler& state)
      {
         state.set_state(FlightManagerState::auto_land);
      };

      constexpr auto set_killed_state = [](StateHandler& state)
      {
         state.set_state(FlightManagerState::killed);
      };

      constexpr auto set_fault_state = [](StateHandler& state)
      {
         state.set_state(FlightManagerState::fault);
      };

      // guards

      constexpr auto health_summary_received = [](StateHandler& state)
      {
         return state.health_summary_received();
      };

      constexpr auto timeout_sensors_readiness = [](StateHandler& state)
      {
         return state.timeout_sensors_readiness();
      };

      constexpr auto timeout_control_readiness = [](StateHandler& state)
      {
         return state.timeout_control_readiness();
      };

      constexpr auto timeout_armed_no_flight = [](StateHandler& state)
      {
         return state.timeout_armed_no_flight();
      };

      constexpr auto timeout_auto_land = [](StateHandler& state)
      {
         return state.timeout_auto_land();
      };

      constexpr auto is_state_change_persistent = [](StateHandler& state)
      {
         return state.is_state_change_persistent();
      };

      constexpr auto sensors_ready = [](StateHandler& state)
      {
         return state.sensors_ready();
      };

      constexpr auto control_ready = [](StateHandler& state)
      {
         return state.control_ready();
      };

      constexpr auto arm = [](StateHandler& state)
      {
         return state.arm();
      };

      constexpr auto disarm = [](StateHandler& state)
      {
         return state.disarm();
      };

      constexpr auto kill = [](StateHandler& state)
      {
         return state.kill();
      };

      constexpr auto manual_mode = [](StateHandler& state)
      {
         return state.manual_mode();
      };

      constexpr auto hover_mode = [](StateHandler& state)
      {
         return state.hover_mode();
      };

      constexpr auto is_health_good = [](StateHandler& state)
      {
         return state.is_health_good();
      };

      constexpr auto is_radio_link_good = [](StateHandler& state)
      {
         return state.is_radio_link_good();
      };

      constexpr auto takeover_requested = [](StateHandler& state)
      {
         return state.takeover_requested();
      };

      constexpr auto landing_complete = [](StateHandler& state)
      {
         return state.landing_complete();
      };

      // events
      static constexpr auto e_tick = event<EventTick>;

      // clang-format off
      return make_transition_table(
          // From State       | Event | Guard                                       | Action                                                      | To State
          *s_init             + e_tick                                              / (read_health_summary, read_radio_input)                     = s_init_checkpoint,
          s_init_checkpoint           [health_summary_received]                     / (set_wait_sensors_state, set_reference_time)                = s_wait_sensors,
          s_init_checkpoint                                                                                                                       = s_init,

          s_wait_sensors      + e_tick                                              / (read_health_summary, get_time)                             = s_wait_sensors_checkpoint,
          s_wait_sensors_checkpoint   [sensors_ready]                               / (start_control, set_wait_control_state, set_reference_time) = s_wait_control,
          s_wait_sensors_checkpoint   [timeout_sensors_readiness]                                                                                 = s_to_fault,
          s_wait_sensors_checkpoint                                                                                                               = s_wait_sensors,

          s_wait_control      + e_tick                                              / (read_health_summary, get_time)                             = s_wait_control_checkpoint,
          s_wait_control_checkpoint   [control_ready]                               / set_disarmed_state                                          = s_disarmed,
          s_wait_control_checkpoint   [timeout_control_readiness]                                                                                 = s_to_fault,
          s_wait_control_checkpoint                                                                                                               = s_wait_control,

          s_disarmed          + e_tick                                              / read_radio_input                                            = s_disarmed_checkpoint,
          s_disarmed_checkpoint       [arm]                                         / set_reference_time                                          = s_arming,
          s_disarmed_checkpoint                                                                                                                   = s_disarmed,

          s_arming            + e_tick                                              / (read_health_summary, read_radio_input, get_time)           = s_arming_checkpoint,
          s_arming_checkpoint         [kill]                                        / (stop_motors, set_killed_state)                             = s_killed,
          s_arming_checkpoint         [disarm || !is_health_good || !is_radio_link_good]                                                          = s_disarmed,
          s_arming_checkpoint         [is_state_change_persistent && is_health_good && is_radio_link_good] / (start_motors, set_armed_state)      = s_armed,
          s_arming_checkpoint                                                                                                                     = s_arming,

          s_armed             + e_tick                                              / (read_health_summary, read_radio_input, get_time)           = s_armed_checkpoint,
          s_armed_checkpoint          [kill]                                        / (stop_motors, set_killed_state)                             = s_killed,
          s_armed_checkpoint          [disarm || timeout_armed_no_flight]           / set_reference_time                                          = s_disarming,
          s_armed_checkpoint          [manual_mode]                                 / set_manual_state                                            = s_manual,
          s_armed_checkpoint          [hover_mode]                                  / set_hover_state                                             = s_hover,
          s_armed_checkpoint          [!is_health_good || !is_radio_link_good]      / stop_motors                                                 = s_to_fault,
          s_armed_checkpoint                                                                                                                      = s_armed,

          s_disarming         + e_tick                                              / (read_health_summary, read_radio_input, get_time)           = s_disarming_checkpoint,
          s_disarming_checkpoint      [kill]                                        / (stop_motors, set_killed_state)                             = s_killed,
          s_disarming_checkpoint      [arm && is_health_good && is_radio_link_good]                                                               = s_armed,
          s_disarming_checkpoint      [is_state_change_persistent]                  / (stop_motors, set_disarmed_state)                           = s_disarmed,
          s_disarming_checkpoint                                                                                                                  = s_disarming,

          s_manual            + e_tick                                              / (read_health_summary, read_radio_input, get_time, publish_manual_setpoint) = s_manual_checkpoint,
          s_manual_checkpoint         [kill]                                        / (stop_motors, set_killed_state)                             = s_killed,
          s_manual_checkpoint         [disarm]                                      / set_reference_time                                          = s_disarming,
          s_manual_checkpoint         [hover_mode]                                  / set_hover_state                                             = s_hover,
          s_manual_checkpoint         [!is_health_good]                             / stop_motors                                                 = s_to_fault,
          s_manual_checkpoint         [!is_radio_link_good]                         / set_auto_land_state                                         = s_auto_land,
          s_manual_checkpoint                                                                                                                     = s_manual,

          s_hover             + e_tick                                              / (read_health_summary, read_radio_input, get_time, publish_hover_setpoint) = s_hover_checkpoint,
          s_hover_checkpoint          [kill]                                        / (stop_motors, set_killed_state)                             = s_killed,
          s_hover_checkpoint          [disarm]                                      / set_reference_time                                          = s_disarming,
          s_hover_checkpoint          [manual_mode]                                 / set_manual_state                                            = s_manual,
          s_hover_checkpoint          [!is_health_good]                             / stop_motors                                                 = s_to_fault,
          s_hover_checkpoint          [!is_radio_link_good]                         / set_auto_land_state                                         = s_auto_land,
          s_hover_checkpoint                                                                                                                      = s_hover,

          s_auto_land         + e_tick                                              / (read_health_summary, read_radio_input, get_time, publish_auto_land_setpoint) = s_auto_land_checkpoint,
          s_auto_land_checkpoint      [kill]                                        / (stop_motors, set_killed_state)                             = s_killed,
          s_auto_land_checkpoint      [!is_health_good || timeout_auto_land]        / stop_motors                                                 = s_to_fault,
          s_auto_land_checkpoint      [landing_complete]                            / (stop_motors, set_disarmed_state)                           = s_disarmed,
          s_auto_land_checkpoint      [is_radio_link_good && takeover_requested]    / set_manual_state                                            = s_manual,
          s_auto_land_checkpoint                                                                                                                  = s_auto_land,

          s_killed            + e_tick                                              / read_radio_input                                            = s_killed_checkpoint,
          s_killed_checkpoint         [!kill]                                       / set_disarmed_state                                          = s_disarmed,

          s_to_fault                                                                / set_fault_state                                             = s_fault

      );
      // clang-format on
   }
};

}   // namespace aeromight_flight

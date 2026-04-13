#pragma once

#include <boost/sml.hpp>

#include "SystemManagerState.hpp"

namespace aeromight_system
{

template <typename StateHandler>
struct SystemManagerStateMachine
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
   static constexpr auto s_imu_calibration         = boost::sml::state<class StateImuCalibration>;
   static constexpr auto s_imu_calib_running       = boost::sml::state<class StateImuCalibrationRunning>;
   static constexpr auto s_imu_calib_checkpoint    = boost::sml::state<class StateImuCalibrationCheckpoint>;
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

      constexpr auto start_control = [](StateHandler& state)
      {
         state.start_control();
      };

      constexpr auto start_imu_calibration = [](StateHandler& state)
      {
         state.start_imu_calibration();
      };

      constexpr auto arm_system = [](StateHandler& state)
      {
         state.arm_system();
      };

      constexpr auto disarm_system = [](StateHandler& state)
      {
         state.disarm_system();
      };

      constexpr auto set_wait_sensors_state = [](StateHandler& state)
      {
         state.set_state(SystemManagerState::wait_sensors);
      };

      constexpr auto set_wait_control_state = [](StateHandler& state)
      {
         state.set_state(SystemManagerState::wait_control);
      };

      constexpr auto set_imu_calibration_state = [](StateHandler& state)
      {
         state.set_state(SystemManagerState::imu_calibration);
      };

      constexpr auto set_disarming_state = [](StateHandler& state)
      {
         state.set_state(SystemManagerState::disarming);
      };

      constexpr auto set_disarmed_state = [](StateHandler& state)
      {
         state.set_state(SystemManagerState::disarmed);
      };

      constexpr auto set_arming_state = [](StateHandler& state)
      {
         state.set_state(SystemManagerState::arming);
      };

      constexpr auto set_armed_state = [](StateHandler& state)
      {
         state.set_state(SystemManagerState::armed);
      };

      constexpr auto set_fault_state = [](StateHandler& state)
      {
         state.set_state(SystemManagerState::fault);
      };

      constexpr auto show_disarm_led = [](StateHandler& state)
      {
         state.show_disarm_led();
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

      constexpr auto run_imu_calibration = [](StateHandler& state)
      {
         return state.imu_calibration();
      };

      constexpr auto imu_calibration_started = [](StateHandler& state)
      {
         return !state.imu_calibration_finished();
      };

      constexpr auto imu_calibration_finished = [](StateHandler& state)
      {
         return state.imu_calibration_finished();
      };

      constexpr auto is_health_good = [](StateHandler& state)
      {
         return state.is_health_good();
      };

      constexpr auto is_radio_link_good = [](StateHandler& state)
      {
         return state.is_radio_link_good();
      };

      // events
      static constexpr auto e_tick = event<EventTick>;

      // clang-format off
      return make_transition_table(
          // From State       | Event  | Guard                                       | Action                                                      | To State
          *s_init             + e_tick                                                                                                             = s_init_checkpoint,
          s_init_checkpoint            [health_summary_received]                     / (set_wait_sensors_state, set_reference_time)                = s_wait_sensors,
          s_init_checkpoint                                                                                                                        = s_init,

          s_wait_sensors      + e_tick                                                                                                             = s_wait_sensors_checkpoint,
          s_wait_sensors_checkpoint    [sensors_ready]                               / (start_control, set_wait_control_state, set_reference_time) = s_wait_control,
          s_wait_sensors_checkpoint    [timeout_sensors_readiness]                                                                                 = s_to_fault,
          s_wait_sensors_checkpoint                                                                                                                = s_wait_sensors,

          s_wait_control      + e_tick                                                                                                             = s_wait_control_checkpoint,
          s_wait_control_checkpoint    [control_ready]                               / set_disarmed_state                                          = s_disarmed,
          s_wait_control_checkpoint    [timeout_control_readiness]                                                                                 = s_to_fault,
          s_wait_control_checkpoint                                                                                                                = s_wait_control,

          s_disarmed          + e_tick                                               / show_disarm_led                                             = s_disarmed_checkpoint,
          s_disarmed_checkpoint        [arm && is_health_good && is_radio_link_good] / (set_arming_state, set_reference_time)                      = s_arming,
          s_disarmed_checkpoint        [run_imu_calibration && is_health_good]       / (start_imu_calibration, set_imu_calibration_state)          = s_imu_calibration,
          s_disarmed_checkpoint                                                                                                                    = s_disarmed,

          s_imu_calibration   + e_tick [imu_calibration_started]                                                                                   = s_imu_calib_running,
          s_imu_calib_running + e_tick                                                                                                             = s_imu_calib_checkpoint,
          s_imu_calib_checkpoint       [imu_calibration_finished]                    / set_disarmed_state                                          = s_disarmed,
          s_imu_calib_checkpoint                                                                                                                   = s_imu_calib_running,

          s_arming            + e_tick                                                                                                             = s_arming_checkpoint,
          s_arming_checkpoint          [disarm || !is_health_good || !is_radio_link_good] / set_disarmed_state                                     = s_disarmed,
          s_arming_checkpoint          [is_state_change_persistent && is_health_good && is_radio_link_good] / (arm_system, set_armed_state)        = s_armed,
          s_arming_checkpoint                                                                                                                      = s_arming,

          s_armed             + e_tick                                                                                                             = s_armed_checkpoint,
          s_armed_checkpoint           [disarm]                                      / (set_disarming_state, set_reference_time)                   = s_disarming,
          s_armed_checkpoint           [!is_health_good || !is_radio_link_good]      / disarm_system                                               = s_to_fault,
          s_armed_checkpoint                                                                                                                       = s_armed,

          s_disarming         + e_tick                                                                                                             = s_disarming_checkpoint,
          s_disarming_checkpoint       [arm && is_health_good && is_radio_link_good]                                                               = s_armed,
          s_disarming_checkpoint       [is_state_change_persistent]                  / (disarm_system, set_disarmed_state)                         = s_disarmed,
          s_disarming_checkpoint                                                                                                                   = s_disarming,

          s_to_fault                                                                 / set_fault_state                                             = s_fault
      );
      // clang-format on
   }
};

}   // namespace aeromight_system

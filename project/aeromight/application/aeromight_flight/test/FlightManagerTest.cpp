#include "aeromight_flight/FlightManager.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <mocks/common/ClockSource.hpp>
#include <mocks/common/Logger.hpp>
#include <mocks/rtos/Notifier.hpp>
#include <mocks/rtos/QueueReceiver.hpp>

#include "boundaries/SharedData.hpp"

class FlightManagerTest : public ::testing::Test
{
   using Setpoints = boundaries::SharedData<aeromight_boundaries::FlightManagerSetpoints>;
   using Actuals   = boundaries::SharedData<aeromight_boundaries::FlightManagerActuals>;

protected:
   void provide_ticks(const uint32_t n)
   {
      uint32_t       current_ms_copy      = current_ms + 1u;
      const uint32_t ms_count_after_ticks = n + current_ms_copy;
      for (; current_ms_copy < ms_count_after_ticks; current_ms_copy++)
      {
         current_ms      = current_ms_copy;
         sys_clock.m_sec = current_ms;
         flight_manager.run_once();
      }
   }

   void provide_health_summary(const std::optional<aeromight_boundaries::HealthSummary> summary)
   {
      EXPECT_CALL(health_summary_queue_mock, receive_latest).WillRepeatedly(::testing::Return(summary));
   }

   void set_good_health_summary()
   {
      health_summary.all_sensors_ready = true;
      health_summary.estimation_ready  = true;
      health_summary.control_ready     = true;
      health_summary.imu_health        = aeromight_boundaries::SubsystemHealth::operational;
      health_summary.barometer_health  = aeromight_boundaries::SubsystemHealth::operational;
      health_summary.estimation_health = aeromight_boundaries::SubsystemHealth::operational;
      health_summary.control_health    = aeromight_boundaries::SubsystemHealth::operational;
   }

   void set_good_radio_input()
   {
      actuals.link_status_ok = true;
      actuals.link_rssi_dbm  = min_good_signal_rssi_dbm + 1.0f;
   }

   void give_arm_command()
   {
      setpoints.state = aeromight_boundaries::FlightArmedState::arm;
      setpoints_storage.update_latest(setpoints, current_ms);
      provide_ticks(1u);
      EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::arming);
   }

   void give_disarm_command()
   {
      setpoints.state = aeromight_boundaries::FlightArmedState::disarm;
      setpoints_storage.update_latest(setpoints, current_ms);
      provide_ticks(1u);
   }

   void give_kill_command()
   {
      setpoints.kill_switch_active = true;
      setpoints_storage.update_latest(setpoints, current_ms);
      provide_ticks(1u);
      EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::killed);
   }

   void give_manual_mode_command()
   {
      setpoints.mode = aeromight_boundaries::FlightMode::stabilized_manual;
      setpoints_storage.update_latest(setpoints, current_ms);
      provide_ticks(1u);
   }

   void give_hover_mode_command()
   {
      setpoints.mode = aeromight_boundaries::FlightMode::altitude_hold;
      setpoints_storage.update_latest(setpoints, current_ms);
      provide_ticks(1u);
   }

   void create_bad_health_summary()
   {
      set_good_radio_input();
      health_summary.flight_health = aeromight_boundaries::FlightHealthStatus::critical;   // bad health
      setpoints_storage.update_latest(setpoints, current_ms);
      actuals_storage.update_latest(actuals, current_ms);
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);
   }

   void create_bad_radio_input()
   {
      set_good_health_summary();
      actuals.link_status_ok = false;
      setpoints_storage.update_latest(setpoints, current_ms);
      actuals_storage.update_latest(actuals, current_ms);
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      flight_manager.run_once();
   }

   void move_to_wait_sensors_state()
   {
      setpoints_storage.update_latest(setpoints, current_ms + 1u);
      actuals_storage.update_latest(actuals, current_ms + 1u);
      health_summary.timestamp_ms = current_ms + 1u;
      provide_health_summary(health_summary);

      provide_ticks(1u);
      EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::wait_sensors);
   }

   void make_sensors_ready()
   {
      health_summary.all_sensors_ready = true;
      health_summary.timestamp_ms      = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);
      EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::wait_control);
   }

   void make_control_ready()
   {
      health_summary.estimation_ready = true;
      health_summary.control_ready    = true;
      health_summary.timestamp_ms     = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);
      EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::disarmed);
   }

   void move_to_armed()
   {
      give_arm_command();

      set_good_health_summary();

      set_good_radio_input();

      for (uint32_t i = 0; i < (min_state_debounce_duration_ms / period_in_ms); i++)
      {
         setpoints_storage.update_latest(setpoints, current_ms);
         actuals_storage.update_latest(actuals, current_ms);
         health_summary.timestamp_ms = current_ms;
         provide_health_summary(health_summary);
         provide_ticks(1u);
      }

      EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::armed);
   }

   static constexpr float    stick_input_deadband_abs       = 0.1f;
   static constexpr float    min_good_signal_rssi_dbm       = -110.0f;
   static constexpr uint32_t period_in_ms                   = 1u;
   static constexpr uint32_t max_age_stale_data_ms          = 5u;
   static constexpr uint32_t min_state_debounce_duration_ms = 3u;
   static constexpr uint32_t timeout_sensors_readiness_ms   = 6u;
   static constexpr uint32_t timeout_control_readiness_ms   = 4u;
   static constexpr uint32_t timeout_auto_land_ms           = 15u;

   mocks::rtos::QueueReceiver<aeromight_boundaries::HealthSummary> health_summary_queue_mock{};
   mocks::rtos::Notifier                                           control_start_notifer_mock{};
   mocks::common::ClockSource                                      sys_clock{};
   Setpoints                                                       setpoints_storage{};
   Actuals                                                         actuals_storage{};
   mocks::common::Logger                                           logger{"flight"};

   aeromight_boundaries::HealthSummary          health_summary{};
   aeromight_boundaries::FlightManagerSetpoints setpoints{};
   aeromight_boundaries::FlightManagerActuals   actuals{};
   uint32_t                                     current_ms{0};

   aeromight_flight::FlightManager<decltype(health_summary_queue_mock),
                                   decltype(control_start_notifer_mock),
                                   decltype(sys_clock), mocks::common::Logger>
       flight_manager{health_summary_queue_mock,
                      control_start_notifer_mock,
                      setpoints_storage,
                      actuals_storage,
                      logger,
                      stick_input_deadband_abs,
                      min_good_signal_rssi_dbm,
                      period_in_ms,
                      max_age_stale_data_ms,
                      min_state_debounce_duration_ms,
                      timeout_sensors_readiness_ms,
                      timeout_control_readiness_ms,
                      timeout_auto_land_ms};
};

TEST_F(FlightManagerTest, initial_state)
{
   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::init);
}

TEST_F(FlightManagerTest, starting_wait_for_sensors)
{
   move_to_wait_sensors_state();
}

TEST_F(FlightManagerTest, timeout_while_waiting_for_sensors_readiness)
{
   move_to_wait_sensors_state();

   const uint32_t ticks_required = timeout_sensors_readiness_ms / period_in_ms;
   provide_ticks(ticks_required);

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::fault);
}

TEST_F(FlightManagerTest, sensors_ready)
{
   move_to_wait_sensors_state();

   make_sensors_ready();
}

TEST_F(FlightManagerTest, timeout_while_waiting_for_control_readiness)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   const uint32_t ticks_required = timeout_control_readiness_ms / period_in_ms;
   provide_ticks(ticks_required);

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::fault);
}

TEST_F(FlightManagerTest, control_ready)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();
}

TEST_F(FlightManagerTest, kill_while_arming)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   // start arming
   give_arm_command();

   // kill
   give_kill_command();
}

TEST_F(FlightManagerTest, disarm_while_arming_due_to_disarm_signal)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   // start arming
   give_arm_command();

   // disarm
   give_disarm_command();
   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::disarmed);
}

TEST_F(FlightManagerTest, fault_while_arming_due_to_stale_health)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   // start arming
   give_arm_command();

   // good but stale health summary
   set_good_health_summary();
   health_summary.timestamp_ms = 0;   // make it zero
   provide_health_summary(health_summary);

   // good and up to date radio input
   set_good_radio_input();

   const uint32_t ticks_needed = (max_age_stale_data_ms - current_ms);
   for (uint32_t i = 0; i < ticks_needed; i++)
   {
      setpoints_storage.update_latest(setpoints, current_ms);
      actuals_storage.update_latest(actuals, current_ms);
      provide_ticks(1u);
   }

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::disarmed);
}

TEST_F(FlightManagerTest, fault_while_arming_due_to_bad_health)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   // valid radio input
   set_good_radio_input();

   // disarm due to imu fault
   {
      // start arming
      give_arm_command();

      // good health summary
      set_good_health_summary();

      // imu not operational
      health_summary.imu_health = aeromight_boundaries::SubsystemHealth::fault;
      setpoints_storage.update_latest(setpoints, current_ms);
      actuals_storage.update_latest(actuals, current_ms);
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);

      EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::disarmed);
   }

   // disarm due to baro fault
   {
      // start arming
      give_arm_command();

      // good health summary
      set_good_health_summary();

      // imu not operational
      health_summary.barometer_health = aeromight_boundaries::SubsystemHealth::fault;
      setpoints_storage.update_latest(setpoints, current_ms);
      actuals_storage.update_latest(actuals, current_ms);
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);

      EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::disarmed);
   }

   // disarm due to estimation fault
   {
      // start arming
      give_arm_command();

      // good health summary
      set_good_health_summary();

      // imu not operational
      health_summary.estimation_health = aeromight_boundaries::SubsystemHealth::fault;
      setpoints_storage.update_latest(setpoints, current_ms);
      actuals_storage.update_latest(actuals, current_ms);
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);

      EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::disarmed);
   }

   // disarm due to control fault
   {
      // start arming
      give_arm_command();

      // good health summary
      set_good_health_summary();

      // imu not operational
      health_summary.control_health = aeromight_boundaries::SubsystemHealth::fault;
      setpoints_storage.update_latest(setpoints, current_ms);
      actuals_storage.update_latest(actuals, current_ms);
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);

      EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::disarmed);
   }

   // disarm due to flight critical fault
   {
      // start arming
      give_arm_command();

      // good health summary
      set_good_health_summary();

      // imu not operational
      health_summary.flight_health = aeromight_boundaries::FlightHealthStatus::critical;
      setpoints_storage.update_latest(setpoints, current_ms);
      actuals_storage.update_latest(actuals, current_ms);
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);

      EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::disarmed);
   }
}

TEST_F(FlightManagerTest, fault_while_arming_due_to_stale_radio_input)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   // start arming
   setpoints.state = aeromight_boundaries::FlightArmedState::arm;
   setpoints_storage.update_latest(setpoints, current_ms);
   provide_ticks(1u);
   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::arming);

   // good but stale radio input
   set_good_radio_input();
   setpoints_storage.update_latest(setpoints, 0);
   actuals_storage.update_latest(actuals, 0);

   // good and up to date health summary
   set_good_health_summary();

   const uint32_t ticks_needed = (max_age_stale_data_ms - current_ms);
   for (uint32_t i = 0; i < ticks_needed; i++)
   {
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);
   }

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::disarmed);
}

TEST_F(FlightManagerTest, fault_while_arming_due_to_bad_radio_input)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   set_good_health_summary();

   // disarm due to link status not ok
   {
      // start arming
      give_arm_command();

      set_good_radio_input();

      // link status not ok
      actuals.link_status_ok = false;
      setpoints_storage.update_latest(setpoints, current_ms);
      actuals_storage.update_latest(actuals, current_ms);
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);

      EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::disarmed);
   }

   // disarm due to low link rssi
   {
      // start arming
      give_arm_command();

      set_good_radio_input();

      // low link rssi
      actuals.link_rssi_dbm = min_good_signal_rssi_dbm - 1.0f;
      setpoints_storage.update_latest(setpoints, current_ms);
      actuals_storage.update_latest(actuals, current_ms);
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);

      EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::disarmed);
   }
}

TEST_F(FlightManagerTest, successfully_armed)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   // @TODO: add check that motors are started
}

TEST_F(FlightManagerTest, kill_while_armed)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   give_kill_command();

   // @TODO: add check that motors are stopped
}

TEST_F(FlightManagerTest, disarm_while_armed)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   give_disarm_command();
   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::disarming);

   const uint32_t ticks_needed = min_state_debounce_duration_ms / period_in_ms;
   provide_ticks(ticks_needed);
   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::disarmed);

   // @TODO: add check that motors are stopped
}

TEST_F(FlightManagerTest, fault_while_armed_due_to_bad_health)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   create_bad_health_summary();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::fault);

   // @TODO: add check that motors are stopped
}

TEST_F(FlightManagerTest, fault_while_armed_due_to_bad_radio_input)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   create_bad_radio_input();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::fault);

   // @TODO: add check that motors are stopped
}

TEST_F(FlightManagerTest, move_to_manual_mode)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   give_manual_mode_command();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::manual_mode);

   // @TODO: add check for manual setpoints
}

TEST_F(FlightManagerTest, kill_while_in_manual_mode)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   give_manual_mode_command();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::manual_mode);

   // @TODO: add check for manual setpoints

   give_kill_command();

   // @TODO: add check that motors are stopped
}

TEST_F(FlightManagerTest, disarm_while_in_manual_mode)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   give_manual_mode_command();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::manual_mode);

   // @TODO: add check for manual setpoints

   give_disarm_command();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::disarming);

   const uint32_t ticks_needed = min_state_debounce_duration_ms / period_in_ms;
   provide_ticks(ticks_needed);
   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::disarmed);
}

TEST_F(FlightManagerTest, move_to_hover_while_in_manual_mode)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   give_manual_mode_command();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::manual_mode);

   give_hover_mode_command();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::hover_mode);

   // @TODO: add check for hover setpoints
}

TEST_F(FlightManagerTest, fault_while_in_manual_mode_due_to_bad_health)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   give_manual_mode_command();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::manual_mode);

   create_bad_health_summary();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::fault);

   // @TODO: add check that motors are stopped
}

TEST_F(FlightManagerTest, auto_land_while_in_manual_mode_due_to_bad_radio_input)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   give_manual_mode_command();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::manual_mode);

   create_bad_radio_input();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::auto_land);
}

TEST_F(FlightManagerTest, move_to_hover_mode)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   give_hover_mode_command();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::hover_mode);

   // @TODO: add check for hover setpoints
}

TEST_F(FlightManagerTest, kill_while_in_hover_mode)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   give_hover_mode_command();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::hover_mode);

   // @TODO: add check for hover setpoints

   give_kill_command();

   // @TODO: add check that motors are stopped
}

TEST_F(FlightManagerTest, disarm_while_in_hover_mode)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   give_hover_mode_command();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::hover_mode);

   // @TODO: add check for hover setpoints

   give_disarm_command();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::disarming);

   const uint32_t ticks_needed = min_state_debounce_duration_ms / period_in_ms;
   provide_ticks(ticks_needed);
   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::disarmed);
}

TEST_F(FlightManagerTest, move_to_manual_mode_while_in_hover_mode)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   give_hover_mode_command();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::hover_mode);

   give_manual_mode_command();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::manual_mode);

   // @TODO: add check for manual setpoints
}

TEST_F(FlightManagerTest, fault_while_in_hover_mode_due_to_bad_health)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   give_hover_mode_command();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::hover_mode);

   create_bad_health_summary();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::fault);

   // @TODO: add check that motors are stopped
}

TEST_F(FlightManagerTest, auto_land_while_in_hover_mode_due_to_bad_radio_input)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   give_hover_mode_command();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::hover_mode);

   create_bad_radio_input();

   EXPECT_EQ(flight_manager.get_state(), aeromight_flight::FlightManagerState::auto_land);
}

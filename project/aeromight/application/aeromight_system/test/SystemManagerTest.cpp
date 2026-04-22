#include "aeromight_system/SystemManager.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <mocks/common/ClockSource.hpp>
#include <mocks/pcb_component/Led.hpp>
#include <mocks/rtos/Notifier.hpp>
#include <mocks/rtos/QueueReceiver.hpp>

#include "boundaries/SharedData.hpp"
#include "logging/Logger.hpp"

class SystemManagerTest : public ::testing::Test
{
protected:
   void provide_ticks(const uint32_t n)
   {
      uint32_t       current_ms_copy      = current_ms + 1u;
      const uint32_t ms_count_after_ticks = n + current_ms_copy;
      for (; current_ms_copy < ms_count_after_ticks; current_ms_copy++)
      {
         current_ms      = current_ms_copy;
         sys_clock.m_sec = current_ms;
         system_manager.run_once();
      }
   }

   void provide_health_summary(const std::optional<aeromight_boundaries::HealthSummary> summary)
   {
      EXPECT_CALL(health_summary_queue_mock, receive_latest).WillRepeatedly(::testing::Return(summary));
   }

   void set_good_health_summary()
   {
      health_summary.imu_operational        = true;
      health_summary.estimation_operational = true;
      health_summary.control_operational    = true;
      health_summary.imu_health             = aeromight_boundaries::SubsystemHealth::operational;
      health_summary.estimation_health      = aeromight_boundaries::SubsystemHealth::operational;
      health_summary.control_health         = aeromight_boundaries::SubsystemHealth::operational;
   }

   void set_good_radio_input()
   {
      link_stats.uplink_quality_pct = params.good_uplink_quality_pct + 1u;
      link_stats.uplink_rssi_1_dbm  = static_cast<int8_t>(params.min_good_signal_rssi_dbm + 1.0f);
   }

   void give_arm_command()
   {
      setpoints.arm = true;
      system_control_setpoints.update_latest(setpoints, current_ms);
      provide_ticks(1u);
      EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::arming);
   }

   void give_disarm_command()
   {
      setpoints.arm = false;
      system_control_setpoints.update_latest(setpoints, current_ms);
      provide_ticks(1u);
   }

   void create_bad_health_summary()
   {
      set_good_radio_input();
      health_summary.control_health = aeromight_boundaries::SubsystemHealth::fault;   // bad health
      system_control_setpoints.update_latest(setpoints, current_ms);
      link_stats_subscriber.update_latest(link_stats, current_ms);
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);
   }

   void create_bad_radio_input()
   {
      set_good_health_summary();
      link_stats.uplink_quality_pct = static_cast<uint8_t>(params.good_uplink_quality_pct - 1u);
      system_control_setpoints.update_latest(setpoints, current_ms);
      link_stats_subscriber.update_latest(link_stats, current_ms);
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      system_manager.run_once();
   }

   void move_to_wait_sensors_state()
   {
      system_control_setpoints.update_latest(setpoints, current_ms + 1u);
      link_stats_subscriber.update_latest(link_stats, current_ms + 1u);
      health_summary.timestamp_ms = current_ms + 1u;
      provide_health_summary(health_summary);

      provide_ticks(1u);
      EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::wait_sensors);
   }

   void make_sensors_ready()
   {
      health_summary.imu_operational = true;
      health_summary.timestamp_ms    = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);
      EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::wait_control);
   }

   void make_control_ready()
   {
      health_summary.estimation_operational = true;
      health_summary.control_operational    = true;
      health_summary.timestamp_ms           = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);
      EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::disarmed);
   }

   void move_to_armed()
   {
      set_good_health_summary();
      provide_health_summary(health_summary);

      set_good_radio_input();
      link_stats_subscriber.update_latest(link_stats, current_ms);

      give_arm_command();

      for (uint32_t i = 0; i < (params.min_state_debounce_duration_ms / params.execution_period_ms); i++)
      {
         system_control_setpoints.update_latest(setpoints, current_ms);
         link_stats_subscriber.update_latest(link_stats, current_ms);
         health_summary.timestamp_ms = current_ms;
         provide_health_summary(health_summary);
         provide_ticks(1u);
      }

      EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::armed);
   }

   aeromight_system::SystemManagerParams params{
       .execution_period_ms            = 1u,
       .stick_input_deadband_abs       = 0.1f,
       .good_uplink_quality_pct        = 50u,
       .min_good_signal_rssi_dbm       = -110.0f,
       .max_age_stale_data_ms          = 5u,
       .min_state_debounce_duration_ms = 3u,
       .timeout_sensors_readiness_ms   = 6u,
       .timeout_control_readiness_ms   = 4u};

   mocks::rtos::QueueReceiver<aeromight_boundaries::HealthSummary>      health_summary_queue_mock{};
   mocks::rtos::Notifier                                                control_start_notifer_mock{};
   mocks::rtos::Notifier                                                imu_start_calibration_notifer_mock{};
   mocks::common::ClockSource                                           sys_clock{};
   mocks::pcb_component::Led                                            led{};
   boundaries::SharedData<aeromight_boundaries::SystemControlSetpoints> system_control_setpoints{};
   boundaries::SharedData<rc::crsf::LinkStats>                          link_stats_subscriber{};
   logging::Logger                                                      logger{"flight"};
   boundaries::SharedData<aeromight_boundaries::SystemState>            system_state{};
   aeromight_boundaries::HealthSummary                                  health_summary{};
   aeromight_boundaries::SystemControlSetpoints                         setpoints{};
   rc::crsf::LinkStats                                                  link_stats{};
   uint32_t                                                             current_ms{0};

   aeromight_system::SystemManager<decltype(health_summary_queue_mock),
                                   decltype(control_start_notifer_mock),
                                   decltype(led),
                                   decltype(sys_clock),
                                   logging::Logger>
       system_manager{health_summary_queue_mock,
                      control_start_notifer_mock,
                      imu_start_calibration_notifer_mock,
                      led,
                      system_state,
                      system_control_setpoints,
                      link_stats_subscriber,
                      logger,
                      params};
};

TEST_F(SystemManagerTest, initial_state)
{
   EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::init);
}

TEST_F(SystemManagerTest, starting_wait_for_sensors)
{
   move_to_wait_sensors_state();
}

TEST_F(SystemManagerTest, timeout_while_waiting_for_sensors_readiness)
{
   move_to_wait_sensors_state();

   const uint32_t ticks_required = params.timeout_sensors_readiness_ms / params.execution_period_ms;
   provide_ticks(ticks_required);

   EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::fault);
}

TEST_F(SystemManagerTest, sensors_ready)
{
   move_to_wait_sensors_state();

   make_sensors_ready();
}

TEST_F(SystemManagerTest, timeout_while_waiting_for_control_readiness)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   const uint32_t ticks_required = params.timeout_control_readiness_ms / params.execution_period_ms;
   provide_ticks(ticks_required);

   EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::fault);
}

TEST_F(SystemManagerTest, control_ready)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();
}

TEST_F(SystemManagerTest, disarm_while_arming_due_to_disarm_signal)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   // start arming
   set_good_health_summary();
   provide_health_summary(health_summary);

   set_good_radio_input();
   link_stats_subscriber.update_latest(link_stats, current_ms);

   give_arm_command();

   // disarm
   give_disarm_command();
   EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::disarmed);
}

TEST_F(SystemManagerTest, fault_while_arming_due_to_stale_health)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   // start arming
   set_good_health_summary();
   provide_health_summary(health_summary);

   set_good_radio_input();
   link_stats_subscriber.update_latest(link_stats, current_ms);

   give_arm_command();

   // good but stale health summary
   set_good_health_summary();
   health_summary.timestamp_ms = 0;   // make it zero
   provide_health_summary(health_summary);

   // good and up to date radio input
   set_good_radio_input();

   const uint32_t ticks_needed = (params.max_age_stale_data_ms - current_ms);
   for (uint32_t i = 0; i < ticks_needed; i++)
   {
      system_control_setpoints.update_latest(setpoints, current_ms);
      link_stats_subscriber.update_latest(link_stats, current_ms);
      provide_ticks(1u);
   }

   EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::disarmed);
}

TEST_F(SystemManagerTest, fault_while_arming_due_to_bad_health)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   // valid radio input
   set_good_radio_input();
   link_stats_subscriber.update_latest(link_stats, current_ms);

   // disarm due to imu fault
   {
      // start arming
      set_good_health_summary();
      provide_health_summary(health_summary);
      give_arm_command();

      // imu not operational
      health_summary.control_health = aeromight_boundaries::SubsystemHealth::fault;
      system_control_setpoints.update_latest(setpoints, current_ms);
      link_stats_subscriber.update_latest(link_stats, current_ms);
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);

      EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::disarmed);
   }

   // disarm due to estimation fault
   {
      // start arming
      set_good_health_summary();
      provide_health_summary(health_summary);
      give_arm_command();

      // imu not operational
      health_summary.estimation_health = aeromight_boundaries::SubsystemHealth::fault;
      system_control_setpoints.update_latest(setpoints, current_ms);
      link_stats_subscriber.update_latest(link_stats, current_ms);
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);

      EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::disarmed);
   }

   // disarm due to control fault
   {
      // start arming
      set_good_health_summary();
      provide_health_summary(health_summary);
      give_arm_command();

      // imu not operational
      health_summary.control_health = aeromight_boundaries::SubsystemHealth::fault;
      system_control_setpoints.update_latest(setpoints, current_ms);
      link_stats_subscriber.update_latest(link_stats, current_ms);
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);

      EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::disarmed);
   }

   // disarm due to flight critical fault
   {
      // start arming
      set_good_health_summary();
      provide_health_summary(health_summary);
      give_arm_command();

      // imu not operational
      health_summary.imu_operational = false;
      health_summary.imu_health      = aeromight_boundaries::SubsystemHealth::fault;
      system_control_setpoints.update_latest(setpoints, current_ms);
      link_stats_subscriber.update_latest(link_stats, current_ms);
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);

      EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::disarmed);
   }
}

TEST_F(SystemManagerTest, fault_while_arming_due_to_stale_radio_input)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   set_good_health_summary();
   provide_health_summary(health_summary);
   set_good_radio_input();
   link_stats_subscriber.update_latest(link_stats, current_ms);

   // start arming
   setpoints.arm = true;
   system_control_setpoints.update_latest(setpoints, current_ms);
   provide_ticks(1u);
   EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::arming);

   // good but stale radio input
   set_good_radio_input();
   system_control_setpoints.update_latest(setpoints, 0);
   link_stats_subscriber.update_latest(link_stats, 0);

   // good and up to date health summary
   set_good_health_summary();

   const uint32_t ticks_needed = (params.max_age_stale_data_ms - current_ms);
   for (uint32_t i = 0; i < ticks_needed; i++)
   {
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);
   }

   EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::disarmed);
}

TEST_F(SystemManagerTest, fault_while_arming_due_to_bad_radio_input)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   set_good_health_summary();
   provide_health_summary(health_summary);
   set_good_radio_input();
   link_stats_subscriber.update_latest(link_stats, current_ms);

   // disarm due to link status not ok
   {
      // start arming
      give_arm_command();

      // link status not ok
      link_stats.uplink_quality_pct = static_cast<uint8_t>(params.good_uplink_quality_pct - 1u);
      system_control_setpoints.update_latest(setpoints, current_ms);
      link_stats_subscriber.update_latest(link_stats, current_ms);
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);

      EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::disarmed);
   }

   // disarm due to low link rssi
   {
      // start arming
      set_good_radio_input();
      link_stats_subscriber.update_latest(link_stats, current_ms);
      give_arm_command();

      // low link rssi
      link_stats.uplink_rssi_1_dbm = static_cast<int8_t>(params.min_good_signal_rssi_dbm - 1.0f);
      system_control_setpoints.update_latest(setpoints, current_ms);
      link_stats_subscriber.update_latest(link_stats, current_ms);
      health_summary.timestamp_ms = current_ms;
      provide_health_summary(health_summary);
      provide_ticks(1u);

      EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::disarmed);
   }
}

TEST_F(SystemManagerTest, successfully_armed)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   // @TODO: add check that motors are started
}

TEST_F(SystemManagerTest, disarm_while_armed)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   give_disarm_command();
   EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::disarming);

   const uint32_t ticks_needed = params.min_state_debounce_duration_ms / params.execution_period_ms;
   provide_ticks(ticks_needed);
   EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::disarmed);

   // @TODO: add check that motors are stopped
}

TEST_F(SystemManagerTest, fault_while_armed_due_to_bad_health)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   create_bad_health_summary();

   EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::fault);

   // @TODO: add check that motors are stopped
}

TEST_F(SystemManagerTest, fault_while_armed_due_to_bad_radio_input)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   create_bad_radio_input();

   EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::fault);

   // @TODO: add check that motors are stopped
}

TEST_F(SystemManagerTest, kill_while_in_manual_mode)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::armed);

   // @TODO: add check for manual setpoints

   // @TODO: add check that motors are stopped
}

TEST_F(SystemManagerTest, disarm_while_in_manual_mode)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::armed);

   // @TODO: add check for manual setpoints

   give_disarm_command();

   EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::disarming);

   const uint32_t ticks_needed = params.min_state_debounce_duration_ms / params.execution_period_ms;
   provide_ticks(ticks_needed);
   EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::disarmed);
}

TEST_F(SystemManagerTest, fault_while_in_manual_mode_due_to_bad_health)
{
   move_to_wait_sensors_state();

   make_sensors_ready();

   make_control_ready();

   move_to_armed();

   EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::armed);

   create_bad_health_summary();

   EXPECT_EQ(system_manager.get_state(), aeromight_system::SystemManagerState::fault);

   // @TODO: add check that motors are stopped
}

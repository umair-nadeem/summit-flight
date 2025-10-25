#include "aeromight_health/HealthMonitoring.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "mocks/common/ClockSource.hpp"
#include "mocks/common/Logger.hpp"
#include "mocks/rtos/QueueSender.hpp"

class HealthMonitoringTest : public testing::Test
{
protected:
   void SetUp() override
   {
      ON_CALL(queue_sender_mock, send_if_possible).WillByDefault(testing::Return(true));
   }

   void provide_ticks(const uint32_t n)
   {
      current_ms++;
      uint32_t       current_ms_copy      = current_ms;
      const uint32_t ms_count_after_ticks = n + current_ms_copy;
      for (; current_ms_copy < ms_count_after_ticks; current_ms_copy++)
      {
         current_ms      = current_ms_copy;
         sys_clock.m_sec = current_ms;
         health_monitoring.run_once();
      }
   }

   void wait_startup()
   {
      const uint32_t tick_required = wait_before_first_summary_update_ms / period_in_ms;
      provide_ticks(tick_required);
   }

   void make_sensors_ready()
   {
      // set imu & barometer operational
      imu_sensor::ImuHealth imu_health{0, imu_sensor::ImuSensorState::operational, 0, true, true, true};
      imu_health_storage.update_latest(imu_health, current_ms);

      barometer_sensor::BarometerHealth baro_health{0, barometer_sensor::BarometerSensorState::operational, 0, 0, true};
      barometer_health_storage.update_latest(baro_health, current_ms);

      current_ms++;
      sys_clock.m_sec = current_ms;
      health_monitoring.run_once();
   }

   void make_estimation_ready()
   {
      aeromight_boundaries::EstimatorHealth estimation_health{0, aeromight_boundaries::EstimatorState::running, 0, true};
      estimation_health_storage.update_latest(estimation_health, current_ms);

      current_ms++;
      sys_clock.m_sec = current_ms;
      health_monitoring.run_once();
   }

   static constexpr std::size_t period_in_ms                             = 1u;
   static constexpr std::size_t wait_before_first_summary_update_ms      = 4u;
   static constexpr std::size_t max_wait_sensors_readiness_ms            = 2u;
   static constexpr std::size_t max_wait_estimation_control_readiness_ms = 3u;
   static constexpr std::size_t max_age_stale_imu_sensor_health_ms       = 3u;
   static constexpr std::size_t max_age_barometer_sensor_health_ms       = 3u;
   static constexpr std::size_t max_age_estimation_health_ms             = 3u;
   static constexpr std::size_t max_age_control_health_ms                = 3u;

   mocks::rtos::QueueSender<aeromight_boundaries::HealthSummary> queue_sender_mock{};
   mocks::common::ClockSource                                    sys_clock{};
   boundaries::SharedData<imu_sensor::ImuHealth>                 imu_health_storage{};
   boundaries::SharedData<barometer_sensor::BarometerHealth>     barometer_health_storage{};
   boundaries::SharedData<aeromight_boundaries::EstimatorHealth> estimation_health_storage{};
   mocks::common::Logger                                         logger_mock{"health_mon"};
   uint32_t                                                      current_ms{0u};

   aeromight_health::HealthMonitoring<decltype(queue_sender_mock),
                                      mocks::common::ClockSource,
                                      decltype(logger_mock)>
       health_monitoring{queue_sender_mock,
                         imu_health_storage,
                         barometer_health_storage,
                         estimation_health_storage,
                         logger_mock,
                         period_in_ms,
                         wait_before_first_summary_update_ms,
                         max_wait_sensors_readiness_ms,
                         max_wait_estimation_control_readiness_ms,
                         max_age_stale_imu_sensor_health_ms,
                         max_age_barometer_sensor_health_ms,
                         max_age_estimation_health_ms,
                         max_age_control_health_ms};
};

TEST_F(HealthMonitoringTest, wait_until_startup)
{
   EXPECT_CALL(queue_sender_mock, send_if_possible).Times(0);
   wait_startup();
}

TEST_F(HealthMonitoringTest, queue_transmission_failure)
{
   EXPECT_CALL(queue_sender_mock, send_if_possible).Times(0);
   wait_startup();

   EXPECT_CALL(queue_sender_mock, send_if_possible).WillRepeatedly(testing::Return(false));
   health_monitoring.run_once();
   health_monitoring.run_once();

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.queue_failure_count, 2u);
}

TEST_F(HealthMonitoringTest, health_summary_passed_to_queue)
{
   EXPECT_CALL(queue_sender_mock, send_if_possible).Times(0);
   wait_startup();

   aeromight_boundaries::HealthSummary health1{};
   health1.timestamp_ms = current_ms;

   EXPECT_CALL(queue_sender_mock, send_if_possible(health1)).Times(1);
   health_monitoring.run_once();

   const aeromight_boundaries::HealthSummary health2 = health_monitoring.get_health_summary();
   EXPECT_EQ(health1, health2);
}

TEST_F(HealthMonitoringTest, timeout_occurs_while_waiting_for_sensor_readiness_imu_ready)
{
   wait_startup();

   // set imu operational
   imu_sensor::ImuHealth imu_health{0, imu_sensor::ImuSensorState::operational, 0, true, true, true};
   imu_health_storage.update_latest(imu_health, current_ms);

   const uint32_t ticks_required = max_wait_sensors_readiness_ms / period_in_ms;
   provide_ticks(ticks_required);

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, false);
   EXPECT_EQ(health.flight_critical_fault, false);
   EXPECT_EQ(health.queue_failure_count, 0);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::fault);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::init);
}

TEST_F(HealthMonitoringTest, timeout_occurs_while_waiting_for_sensor_readiness_baro_ready)
{
   wait_startup();
   
   // set barometer operational
   barometer_sensor::BarometerHealth baro_health{0, barometer_sensor::BarometerSensorState::operational, 0, 0, true};
   barometer_health_storage.update_latest(baro_health, current_ms);
   
   const uint32_t ticks_required = max_wait_sensors_readiness_ms / period_in_ms;
   provide_ticks(ticks_required);

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, false);
   EXPECT_EQ(health.flight_critical_fault, false);
   EXPECT_EQ(health.queue_failure_count, 0);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::fault);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::init);
}

TEST_F(HealthMonitoringTest, timeout_occurs_while_waiting_for_sensor_readiness_none_ready)
{
   wait_startup();
   
   // set imu & barometer operational
   imu_sensor::ImuHealth imu_health{0, imu_sensor::ImuSensorState::operational, 0, true, true, false};                   // self-test failed
   imu_health_storage.update_latest(imu_health, current_ms);
   
   barometer_sensor::BarometerHealth baro_health{0, barometer_sensor::BarometerSensorState::operational, 0, 0, false};   // setup failed
   barometer_health_storage.update_latest(baro_health, current_ms);
   
   const uint32_t ticks_required = max_wait_sensors_readiness_ms / period_in_ms;
   provide_ticks(ticks_required);
   
   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, false);
   EXPECT_EQ(health.flight_critical_fault, false);
   EXPECT_EQ(health.queue_failure_count, 0);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::fault);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::fault);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::init);
}

TEST_F(HealthMonitoringTest, wait_sensor_readiness_both_ready)
{
   wait_startup();

   make_sensors_ready();

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, true);
   EXPECT_EQ(health.estimation_ready, false);
   EXPECT_EQ(health.flight_critical_fault, false);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::init);
}

TEST_F(HealthMonitoringTest, timeout_occurs_while_waiting_for_estimation_readiness)
{
   wait_startup();

   make_sensors_ready();

   aeromight_boundaries::EstimatorHealth estimation_health{0, aeromight_boundaries::EstimatorState::running, 0, false};   // reference pressure not acquired
   estimation_health_storage.update_latest(estimation_health, current_ms);

   const uint32_t ticks_required = max_wait_estimation_control_readiness_ms / period_in_ms;
   provide_ticks(ticks_required);

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, true);
   EXPECT_EQ(health.estimation_ready, false);
   EXPECT_EQ(health.flight_critical_fault, false);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::fault);
}

TEST_F(HealthMonitoringTest, wait_estimation_readiness)
{
   wait_startup();

   make_sensors_ready();

   make_estimation_ready();

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, true);
   EXPECT_EQ(health.estimation_ready, true);
   EXPECT_EQ(health.flight_critical_fault, false);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
}

TEST_F(HealthMonitoringTest, stale_imu_data_causes_degraded_health)
{
   wait_startup();

   make_sensors_ready();

   make_estimation_ready();

   // update baro and estimation health status
   current_ms += max_age_stale_imu_sensor_health_ms;

   barometer_sensor::BarometerHealth baro_health{0, barometer_sensor::BarometerSensorState::operational, 0, 0, true};
   barometer_health_storage.update_latest(baro_health, current_ms);

   aeromight_boundaries::EstimatorHealth estimation_health{0, aeromight_boundaries::EstimatorState::running, 0, true};
   estimation_health_storage.update_latest(estimation_health, current_ms);

   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, true);
   EXPECT_EQ(health.estimation_ready, true);
   EXPECT_EQ(health.flight_critical_fault, false);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::degraded);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
}

TEST_F(HealthMonitoringTest, stale_baro_data_causes_degraded_health)
{
   wait_startup();

   make_sensors_ready();

   make_estimation_ready();

   // update imu and estimation health status
   current_ms += max_age_stale_imu_sensor_health_ms;

   imu_sensor::ImuHealth imu_health{0, imu_sensor::ImuSensorState::operational, 0, true, true, true};
   imu_health_storage.update_latest(imu_health, current_ms);

   aeromight_boundaries::EstimatorHealth estimation_health{0, aeromight_boundaries::EstimatorState::running, 0, true};
   estimation_health_storage.update_latest(estimation_health, current_ms);

   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, true);
   EXPECT_EQ(health.estimation_ready, true);
   EXPECT_EQ(health.flight_critical_fault, false);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::degraded);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
}

TEST_F(HealthMonitoringTest, stale_estimation_data_causes_degraded_health)
{
   wait_startup();

   make_sensors_ready();

   make_estimation_ready();

   // update imu and baro health status
   current_ms += max_age_stale_imu_sensor_health_ms;

   imu_sensor::ImuHealth imu_health{0, imu_sensor::ImuSensorState::operational, 0, true, true, true};
   imu_health_storage.update_latest(imu_health, current_ms);

   barometer_sensor::BarometerHealth baro_health{0, barometer_sensor::BarometerSensorState::operational, 0, 0, true};
   barometer_health_storage.update_latest(baro_health, current_ms);

   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, true);
   EXPECT_EQ(health.estimation_ready, true);
   EXPECT_EQ(health.flight_critical_fault, false);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::degraded);
}

TEST_F(HealthMonitoringTest, imu_non_zero_read_failures_cause_degraded_health)
{
   wait_startup();

   make_sensors_ready();

   make_estimation_ready();

   imu_sensor::ImuHealth imu_health{0, imu_sensor::ImuSensorState::operational, 1u, true, true, true};   // 1 read failure
   imu_health_storage.update_latest(imu_health, current_ms);

   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, true);
   EXPECT_EQ(health.estimation_ready, true);
   EXPECT_EQ(health.flight_critical_fault, false);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::degraded);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
}

TEST_F(HealthMonitoringTest, baro_non_zero_read_failures_cause_degraded_health)
{
   wait_startup();

   make_sensors_ready();

   make_estimation_ready();

   barometer_sensor::BarometerHealth baro_health{0, barometer_sensor::BarometerSensorState::operational, 1u, 0, true};   // 1 read failure
   barometer_health_storage.update_latest(baro_health, current_ms);

   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, true);
   EXPECT_EQ(health.estimation_ready, true);
   EXPECT_EQ(health.flight_critical_fault, false);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::degraded);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
}

TEST_F(HealthMonitoringTest, imu_failure_causes_flight_critical_fault)
{
   wait_startup();

   make_sensors_ready();

   make_estimation_ready();

   const auto error_types = std::array{imu_sensor::ImuSensorError::bus_error,
                                       imu_sensor::ImuSensorError::bus_error,
                                       imu_sensor::ImuSensorError::id_mismatch_error,
                                       imu_sensor::ImuSensorError::config_mismatch_error,
                                       imu_sensor::ImuSensorError::data_pattern_error,
                                       imu_sensor::ImuSensorError::out_of_range_data_error,
                                       imu_sensor::ImuSensorError::non_stationary_calibration_error,
                                       imu_sensor::ImuSensorError::unstable_accel_error,
                                       imu_sensor::ImuSensorError::unstable_gyro_error};

   // good health for baro
   barometer_sensor::BarometerHealth baro_health{0, barometer_sensor::BarometerSensorState::operational, 0, 0, true};

   // good health for estimation
   aeromight_boundaries::EstimatorHealth estimation_health{0, aeromight_boundaries::EstimatorState::running, 0, true};

   // degraded health for imu
   imu_sensor::ImuHealth imu_health{0, imu_sensor::ImuSensorState::operational, 1u, true, true, true};   // 1 read failure

   aeromight_boundaries::HealthSummary health{};

   // if causes for both degradation and fault are present, fault will take priority
   for (const auto& error : error_types)
   {
      imu_health.error.reset();
      imu_health.error.set(static_cast<uint8_t>(error));
      imu_health_storage.update_latest(imu_health, current_ms);
      barometer_health_storage.update_latest(baro_health, current_ms);          // keep providing good baro health status
      estimation_health_storage.update_latest(estimation_health, current_ms);   // keep providing good estimation health status
      current_ms++;
      sys_clock.m_sec = current_ms;
      health_monitoring.run_once();

      health = health_monitoring.get_health_summary();
      EXPECT_EQ(health.timestamp_ms, current_ms);
      EXPECT_EQ(health.all_sensors_ready, true);
      EXPECT_EQ(health.estimation_ready, true);
      EXPECT_EQ(health.flight_critical_fault, true);   // imu failure -> flight critical
      EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::fault);
      EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
      EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
   }

   // error is gone but state is failure -> state is fault
   imu_health.error.reset();
   imu_health.state = imu_sensor::ImuSensorState::failure;
   imu_health_storage.update_latest(imu_health, current_ms);
   barometer_health_storage.update_latest(baro_health, current_ms);          // keep providing good baro health status
   estimation_health_storage.update_latest(estimation_health, current_ms);   // keep providing good estimation health status
   current_ms++;
   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, true);
   EXPECT_EQ(health.estimation_ready, true);
   EXPECT_EQ(health.flight_critical_fault, true);   // imu failure -> flight critical
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::fault);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);

   // removing causes of fault -> switches health state to degraded due to 1 read failure
   imu_health.state = imu_sensor::ImuSensorState::operational;
   imu_health_storage.update_latest(imu_health, current_ms);
   barometer_health_storage.update_latest(baro_health, current_ms);          // keep providing good baro health status
   estimation_health_storage.update_latest(estimation_health, current_ms);   // keep providing good estimation health status
   current_ms++;
   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, true);
   EXPECT_EQ(health.estimation_ready, true);
   EXPECT_EQ(health.flight_critical_fault, false);                                  // no longer flight critical
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::degraded);   // still degraded
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);

   // removing cause of health degradation
   imu_health.read_failure_count = 0;
   imu_health_storage.update_latest(imu_health, current_ms);
   barometer_health_storage.update_latest(baro_health, current_ms);          // keep providing good baro health status
   estimation_health_storage.update_latest(estimation_health, current_ms);   // keep providing good estimation health status
   current_ms++;
   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, true);
   EXPECT_EQ(health.estimation_ready, true);
   EXPECT_EQ(health.flight_critical_fault, false);   // no longer flight critical
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
}

TEST_F(HealthMonitoringTest, baro_failure_causes_flight_critical_fault)
{
   wait_startup();

   make_sensors_ready();

   make_estimation_ready();

   const auto error_types = std::array{barometer_sensor::BarometerSensorError::bus_error,
                                       barometer_sensor::BarometerSensorError::id_mismatch_error,
                                       barometer_sensor::BarometerSensorError::config_mismatch_error,
                                       barometer_sensor::BarometerSensorError::coefficients_pattern_error,
                                       barometer_sensor::BarometerSensorError::sensor_error,
                                       barometer_sensor::BarometerSensorError::data_pattern_error,
                                       barometer_sensor::BarometerSensorError::out_of_range_data_error};

   // good health for imu
   imu_sensor::ImuHealth imu_health{0, imu_sensor::ImuSensorState::operational, 0u, true, true, true};

   // good health for estimation
   aeromight_boundaries::EstimatorHealth estimation_health{0, aeromight_boundaries::EstimatorState::running, 0, true};

   // degraded health for baro
   barometer_sensor::BarometerHealth baro_health{0, barometer_sensor::BarometerSensorState::operational, 1u, 0, true};   // 1 read failure

   aeromight_boundaries::HealthSummary health{};

   // if causes for both degradation and fault are present, fault will take priority
   for (const auto& error : error_types)
   {
      baro_health.error.reset();
      baro_health.error.set(static_cast<uint8_t>(error));
      barometer_health_storage.update_latest(baro_health, current_ms);
      imu_health_storage.update_latest(imu_health, current_ms);                 // keep providing good imu health status
      estimation_health_storage.update_latest(estimation_health, current_ms);   // keep providing good estimation health status
      current_ms++;
      sys_clock.m_sec = current_ms;
      health_monitoring.run_once();

      health = health_monitoring.get_health_summary();
      EXPECT_EQ(health.timestamp_ms, current_ms);
      EXPECT_EQ(health.all_sensors_ready, true);
      EXPECT_EQ(health.estimation_ready, true);
      EXPECT_EQ(health.flight_critical_fault, true);   // baro failure -> flight critical
      EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
      EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::fault);
      EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
   }

   // error is gone but state is failure -> state is fault
   baro_health.error.reset();
   baro_health.state = barometer_sensor::BarometerSensorState::failure;
   barometer_health_storage.update_latest(baro_health, current_ms);
   imu_health_storage.update_latest(imu_health, current_ms);                 // keep providing good imu health status
   estimation_health_storage.update_latest(estimation_health, current_ms);   // keep providing good estimation health status
   current_ms++;
   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, true);
   EXPECT_EQ(health.estimation_ready, true);
   EXPECT_EQ(health.flight_critical_fault, true);   // baro failure -> flight critical
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::fault);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);

   // removing causes of fault -> switches health state to degraded due to 1 read failure
   baro_health.state = barometer_sensor::BarometerSensorState::operational;
   barometer_health_storage.update_latest(baro_health, current_ms);
   imu_health_storage.update_latest(imu_health, current_ms);                 // keep providing good imu health status
   estimation_health_storage.update_latest(estimation_health, current_ms);   // keep providing good estimation health status
   current_ms++;
   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, true);
   EXPECT_EQ(health.estimation_ready, true);
   EXPECT_EQ(health.flight_critical_fault, false);                                        // no longer flight critical
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::degraded);   // still degraded
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);

   // removing cause of health degradation
   baro_health.read_failure_count = 0;
   barometer_health_storage.update_latest(baro_health, current_ms);
   imu_health_storage.update_latest(imu_health, current_ms);                 // keep providing good imu health status
   estimation_health_storage.update_latest(estimation_health, current_ms);   // keep providing good estimation health status
   current_ms++;
   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, true);
   EXPECT_EQ(health.estimation_ready, true);
   EXPECT_EQ(health.flight_critical_fault, false);   // no longer flight critical
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
}

TEST_F(HealthMonitoringTest, estimation_failure_causes_flight_critical_fault)
{
   wait_startup();

   make_sensors_ready();

   make_estimation_ready();

   const auto error_types = std::array{aeromight_boundaries::EstimatorHealth::Error::reference_pressure_estimate_timeout,
                                       aeromight_boundaries::EstimatorHealth::Error::reference_pressure_implausible,
                                       aeromight_boundaries::EstimatorHealth::Error::stale_imu_sensor_data,
                                       aeromight_boundaries::EstimatorHealth::Error::stale_baro_sensor_data,
                                       aeromight_boundaries::EstimatorHealth::Error::missing_valid_imu_data,
                                       aeromight_boundaries::EstimatorHealth::Error::missing_valid_baro_data};

   // good health for imu
   imu_sensor::ImuHealth imu_health{0, imu_sensor::ImuSensorState::operational, 0u, true, true, true};

   // good health for baro
   barometer_sensor::BarometerHealth baro_health{0, barometer_sensor::BarometerSensorState::operational, 0, 0, true};

   // degraded health for estimation
   aeromight_boundaries::EstimatorHealth estimation_health{0, aeromight_boundaries::EstimatorState::running, 0, true};

   aeromight_boundaries::HealthSummary health{};

   // if causes for both degradation and fault are present, fault will take priority
   for (const auto& error : error_types)
   {
      estimation_health.error.reset();
      estimation_health.error.set(static_cast<uint8_t>(error));
      estimation_health_storage.update_latest(estimation_health, current_ms);
      imu_health_storage.update_latest(imu_health, current_ms);          // keep providing good imu health status
      barometer_health_storage.update_latest(baro_health, current_ms);   // keep providing good baro health status
      current_ms++;
      sys_clock.m_sec = current_ms;
      health_monitoring.run_once();

      health = health_monitoring.get_health_summary();
      EXPECT_EQ(health.timestamp_ms, current_ms);
      EXPECT_EQ(health.all_sensors_ready, true);
      EXPECT_EQ(health.estimation_ready, true);
      EXPECT_EQ(health.flight_critical_fault, true);   // estimation failure -> flight critical
      EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
      EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
      EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::fault);
   }

   // error is gone but state is failure -> state is fault
   estimation_health.error.reset();
   estimation_health.state = aeromight_boundaries::EstimatorState::fault;
   estimation_health_storage.update_latest(estimation_health, current_ms);
   imu_health_storage.update_latest(imu_health, current_ms);          // keep providing good imu health status
   barometer_health_storage.update_latest(baro_health, current_ms);   // keep providing good baro health status
   current_ms++;
   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, true);
   EXPECT_EQ(health.estimation_ready, true);
   EXPECT_EQ(health.flight_critical_fault, true);   // estimation failure -> flight critical
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::fault);

   // removing cause of fault
   estimation_health.state = aeromight_boundaries::EstimatorState::running;
   estimation_health_storage.update_latest(estimation_health, current_ms);
   imu_health_storage.update_latest(imu_health, current_ms);          // keep providing good imu health status
   barometer_health_storage.update_latest(baro_health, current_ms);   // keep providing good baro health status
   current_ms++;
   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.all_sensors_ready, true);
   EXPECT_EQ(health.estimation_ready, true);
   EXPECT_EQ(health.flight_critical_fault, false);   // no longer flight critical
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
}

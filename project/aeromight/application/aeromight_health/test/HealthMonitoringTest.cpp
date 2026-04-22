#include "aeromight_health/HealthMonitoring.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "logging/Logger.hpp"
#include "mocks/common/ClockSource.hpp"
#include "mocks/rtos/QueueSender.hpp"

class BatteryMock
{
public:
   MOCK_METHOD(void, execute, (), ());
   MOCK_METHOD(power::battery::BatteryStatus, get_status, (), ());
};

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

   void make_sensors_ready()
   {
      // set imu & barometer operational
      imu::ImuStatus imu_health{0, true};
      imu_health_subscriber.update_latest(imu_health, current_ms);

      barometer::BarometerStatus baro_health{true};
      barometer_health_subscriber.update_latest(baro_health, current_ms);

      current_ms++;
      sys_clock.m_sec = current_ms;
      health_monitoring.run_once();
   }

   void make_estimation_and_control_ready()
   {
      aeromight_boundaries::EstimatorStatus estimation_health{true, 0};
      estimation_health_subscriber.update_latest(estimation_health, current_ms);

      aeromight_boundaries::ControlStatus control_health{true, 0};
      control_health_subscriber.update_latest(control_health, current_ms);

      current_ms++;
      sys_clock.m_sec = current_ms;
      health_monitoring.run_once();
   }

   aeromight_health::HealthMonitoringParams params{
       .execution_period_ms                = 1u,
       .max_age_stale_imu_sensor_health_ms = 3u,
       .max_age_barometer_sensor_health_ms = 3u,
       .max_age_estimation_health_ms       = 3u,
       .max_age_control_health_ms          = 3u,
       .evaluate_barometer_health          = true};

   BatteryMock                                                   battery_mock{};
   mocks::rtos::QueueSender<aeromight_boundaries::HealthSummary> queue_sender_mock{};
   mocks::common::ClockSource                                    sys_clock{};
   boundaries::SharedData<power::battery::BatteryStatus>         battery_status_publisher{};
   boundaries::SharedData<imu::ImuStatus>                        imu_health_subscriber{};
   boundaries::SharedData<barometer::BarometerStatus>            barometer_health_subscriber{};
   boundaries::SharedData<aeromight_boundaries::EstimatorStatus> estimation_health_subscriber{};
   boundaries::SharedData<aeromight_boundaries::ControlStatus>   control_health_subscriber{};
   logging::Logger                                               logger_mock{"health_mon"};
   uint32_t                                                      current_ms{0u};

   aeromight_health::HealthMonitoring<decltype(battery_mock),
                                      decltype(queue_sender_mock),
                                      mocks::common::ClockSource,
                                      decltype(logger_mock)>
       health_monitoring{battery_mock,
                         queue_sender_mock,
                         battery_status_publisher,
                         imu_health_subscriber,
                         barometer_health_subscriber,
                         estimation_health_subscriber,
                         control_health_subscriber,
                         logger_mock,
                         params};
};

TEST_F(HealthMonitoringTest, wait_until_startup)
{
   EXPECT_CALL(queue_sender_mock, send_if_possible).Times(0);
}

TEST_F(HealthMonitoringTest, queue_transmission_failure)
{
   EXPECT_CALL(queue_sender_mock, send_if_possible).Times(0);

   EXPECT_CALL(queue_sender_mock, send_if_possible).WillRepeatedly(testing::Return(false));
   health_monitoring.run_once();
   health_monitoring.run_once();

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.total_queue_failure_count, 2u);
}

TEST_F(HealthMonitoringTest, health_summary_passed_to_queue)
{
   EXPECT_CALL(queue_sender_mock, send_if_possible).Times(0);

   aeromight_boundaries::HealthSummary health1{};
   health1.timestamp_ms = current_ms;

   EXPECT_CALL(queue_sender_mock, send_if_possible(health1)).Times(1);
   health_monitoring.run_once();

   const aeromight_boundaries::HealthSummary health2 = health_monitoring.get_health_summary();
   EXPECT_EQ(health1, health2);
}

TEST_F(HealthMonitoringTest, wait_sensor_readiness_both_ready)
{

   make_sensors_ready();

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.imu_operational, true);
   EXPECT_EQ(health.estimation_operational, false);
   EXPECT_EQ(health.control_operational, false);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::init);
   EXPECT_EQ(health.control_health, aeromight_boundaries::SubsystemHealth::init);
}

TEST_F(HealthMonitoringTest, wait_estimation_ready)
{

   make_sensors_ready();

   aeromight_boundaries::EstimatorStatus estimation_health{true, 0};
   estimation_health_subscriber.update_latest(estimation_health, current_ms);

   current_ms++;
   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.imu_operational, true);
   EXPECT_EQ(health.estimation_operational, true);
   EXPECT_EQ(health.control_operational, false);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.control_health, aeromight_boundaries::SubsystemHealth::init);
}

TEST_F(HealthMonitoringTest, stale_imu_data_causes_degraded_health)
{

   make_sensors_ready();

   make_estimation_and_control_ready();

   // update baro, control and estimation health status
   current_ms += params.max_age_stale_imu_sensor_health_ms;

   barometer::BarometerStatus baro_health{true};
   barometer_health_subscriber.update_latest(baro_health, current_ms);

   aeromight_boundaries::EstimatorStatus estimation_health{true, 0};
   estimation_health_subscriber.update_latest(estimation_health, current_ms);

   aeromight_boundaries::ControlStatus control_health{true, 0};
   control_health_subscriber.update_latest(control_health, current_ms);

   const uint32_t ticks_required = (params.max_age_stale_imu_sensor_health_ms / params.execution_period_ms) - 1u;   // one tick to make estimation ready
   provide_ticks(ticks_required);

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.imu_operational, true);
   EXPECT_EQ(health.estimation_operational, true);
   EXPECT_EQ(health.control_operational, true);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::degraded);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.control_health, aeromight_boundaries::SubsystemHealth::operational);
}

TEST_F(HealthMonitoringTest, stale_baro_data_does_not_cause_degraded_health)
{

   make_sensors_ready();

   make_estimation_and_control_ready();

   // update imu, estimation and control health status
   current_ms += params.max_age_stale_imu_sensor_health_ms;

   imu::ImuStatus imu_health{0, true};
   imu_health_subscriber.update_latest(imu_health, current_ms);

   aeromight_boundaries::EstimatorStatus estimation_health{true, 0};
   estimation_health_subscriber.update_latest(estimation_health, current_ms);

   aeromight_boundaries::ControlStatus control_health{true, 0};
   control_health_subscriber.update_latest(control_health, current_ms);

   const uint32_t ticks_required = (params.max_age_stale_imu_sensor_health_ms / params.execution_period_ms) - 1u;   // one tick to make estimation ready
   provide_ticks(ticks_required);

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.imu_operational, true);
   EXPECT_EQ(health.estimation_operational, true);
   EXPECT_EQ(health.control_operational, true);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::degraded);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.control_health, aeromight_boundaries::SubsystemHealth::operational);
}

TEST_F(HealthMonitoringTest, stale_estimation_data_causes_degraded_health)
{

   make_sensors_ready();

   make_estimation_and_control_ready();

   // update imu and baro health status
   current_ms += params.max_age_stale_imu_sensor_health_ms;

   imu::ImuStatus imu_health{0, true};
   imu_health_subscriber.update_latest(imu_health, current_ms);

   barometer::BarometerStatus baro_health{true};
   barometer_health_subscriber.update_latest(baro_health, current_ms);

   aeromight_boundaries::ControlStatus control_health{true, 0};
   control_health_subscriber.update_latest(control_health, current_ms);

   const uint32_t ticks_required = (params.max_age_stale_imu_sensor_health_ms / params.execution_period_ms) - 1u;   // one tick to make estimation ready
   provide_ticks(ticks_required);

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.imu_operational, true);
   EXPECT_EQ(health.estimation_operational, true);
   EXPECT_EQ(health.control_operational, true);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::degraded);
   EXPECT_EQ(health.control_health, aeromight_boundaries::SubsystemHealth::operational);
}

TEST_F(HealthMonitoringTest, imu_non_zero_read_failures_cause_degraded_health)
{

   make_sensors_ready();

   make_estimation_and_control_ready();

   imu::ImuStatus imu_health{1u, true};   // 1 read failure
   imu_health_subscriber.update_latest(imu_health, current_ms);

   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.imu_operational, true);
   EXPECT_EQ(health.estimation_operational, true);
   EXPECT_EQ(health.control_operational, true);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::degraded);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.control_health, aeromight_boundaries::SubsystemHealth::operational);
}

TEST_F(HealthMonitoringTest, baro_non_zero_read_failures_does_not_cause_degraded_health)
{

   make_sensors_ready();

   make_estimation_and_control_ready();

   barometer::BarometerStatus baro_health{true};   // 1 read failure
   barometer_health_subscriber.update_latest(baro_health, current_ms);

   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   const aeromight_boundaries::HealthSummary health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.imu_operational, true);
   EXPECT_EQ(health.estimation_operational, true);
   EXPECT_EQ(health.control_operational, true);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::degraded);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.control_health, aeromight_boundaries::SubsystemHealth::operational);
}

TEST_F(HealthMonitoringTest, imu_failure_causes_flight_critical_fault)
{

   make_sensors_ready();

   make_estimation_and_control_ready();

   const auto error_types = std::array{imu::ImuError::non_stationary_calibration_error,
                                       imu::ImuError::unstable_accel_error,
                                       imu::ImuError::unstable_gyro_error};

   // good health for baro
   barometer::BarometerStatus baro_health{true};

   // good health for estimation
   aeromight_boundaries::EstimatorStatus estimation_health{true, 0};
   aeromight_boundaries::ControlStatus   control_health{true, 0};

   // degraded health for imu
   imu::ImuStatus imu_health{1u, true};   // 1 read failure

   aeromight_boundaries::HealthSummary health{};

   // if causes for both degradation and fault are present, fault will take priority
   for (const auto& error : error_types)
   {
      imu_health.error.reset();
      imu_health.error.set(static_cast<types::ErrorBitsType>(error));
      imu_health_subscriber.update_latest(imu_health, current_ms);
      barometer_health_subscriber.update_latest(baro_health, current_ms);          // keep providing good baro health status
      estimation_health_subscriber.update_latest(estimation_health, current_ms);   // keep providing good estimation health status
      control_health_subscriber.update_latest(control_health, current_ms);         // keep providing good control health status
      current_ms++;
      sys_clock.m_sec = current_ms;
      health_monitoring.run_once();

      health = health_monitoring.get_health_summary();
      EXPECT_EQ(health.timestamp_ms, current_ms);
      EXPECT_EQ(health.imu_operational, true);
      EXPECT_EQ(health.estimation_operational, true);
      EXPECT_EQ(health.control_operational, true);
      EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::fault);
      EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
      EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
      EXPECT_EQ(health.control_health, aeromight_boundaries::SubsystemHealth::operational);
   }

   // error is gone but state is failure -> state is fault
   imu_health.error.reset();
   imu_health_subscriber.update_latest(imu_health, current_ms);
   barometer_health_subscriber.update_latest(baro_health, current_ms);          // keep providing good baro health status
   estimation_health_subscriber.update_latest(estimation_health, current_ms);   // keep providing good estimation health status
   control_health_subscriber.update_latest(control_health, current_ms);         // keep providing good control health status
   current_ms++;
   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.imu_operational, true);
   EXPECT_EQ(health.estimation_operational, true);
   EXPECT_EQ(health.control_operational, true);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::fault);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.control_health, aeromight_boundaries::SubsystemHealth::operational);

   // removing causes of fault -> switches health state to degraded due to 1 read failure
   imu_health_subscriber.update_latest(imu_health, current_ms);
   barometer_health_subscriber.update_latest(baro_health, current_ms);          // keep providing good baro health status
   estimation_health_subscriber.update_latest(estimation_health, current_ms);   // keep providing good estimation health status
   control_health_subscriber.update_latest(control_health, current_ms);         // keep providing good control health status
   current_ms++;
   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.imu_operational, true);
   EXPECT_EQ(health.estimation_operational, true);
   EXPECT_EQ(health.control_operational, true);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::degraded);   // still degraded
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.control_health, aeromight_boundaries::SubsystemHealth::operational);

   // removing cause of health degradation
   imu_health_subscriber.update_latest(imu_health, current_ms);
   barometer_health_subscriber.update_latest(baro_health, current_ms);          // keep providing good baro health status
   estimation_health_subscriber.update_latest(estimation_health, current_ms);   // keep providing good estimation health status
   control_health_subscriber.update_latest(control_health, current_ms);         // keep providing good control health status
   current_ms++;
   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.imu_operational, true);
   EXPECT_EQ(health.estimation_operational, true);
   EXPECT_EQ(health.control_operational, true);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.control_health, aeromight_boundaries::SubsystemHealth::operational);
}

TEST_F(HealthMonitoringTest, estimation_failure_causes_flight_critical_fault)
{

   make_sensors_ready();

   make_estimation_and_control_ready();

   const auto error_types = std::array{aeromight_boundaries::EstimatorStatus::Error::stale_imu_sensor_data,   // non-critical errors only
                                       aeromight_boundaries::EstimatorStatus::Error::stale_baro_sensor_data,
                                       aeromight_boundaries::EstimatorStatus::Error::missing_valid_imu_data,
                                       aeromight_boundaries::EstimatorStatus::Error::missing_valid_baro_data};

   // good health for imu
   imu::ImuStatus imu_health{0, true};

   // good health for baro
   barometer::BarometerStatus baro_health{true};

   // degraded health for estimation
   aeromight_boundaries::EstimatorStatus estimation_health{true, 0};
   aeromight_boundaries::ControlStatus   control_health{true, 0};

   aeromight_boundaries::HealthSummary health{};

   // if causes for both degradation and fault are present, fault will take priority
   for (const auto& error : error_types)
   {
      estimation_health.error.reset();
      estimation_health.error.set(static_cast<types::ErrorBitsType>(error));
      estimation_health_subscriber.update_latest(estimation_health, current_ms);
      imu_health_subscriber.update_latest(imu_health, current_ms);           // keep providing good imu health status
      barometer_health_subscriber.update_latest(baro_health, current_ms);    // keep providing good baro health status
      control_health_subscriber.update_latest(control_health, current_ms);   // keep providing good control health status
      current_ms++;
      sys_clock.m_sec = current_ms;
      health_monitoring.run_once();

      health = health_monitoring.get_health_summary();
      EXPECT_EQ(health.timestamp_ms, current_ms);
      EXPECT_EQ(health.imu_operational, true);
      EXPECT_EQ(health.estimation_operational, true);
      EXPECT_EQ(health.control_operational, true);
      EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
      EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
      EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::fault);   // estimation failure -> flight critical
      EXPECT_EQ(health.control_health, aeromight_boundaries::SubsystemHealth::operational);
   }

   // error is gone but state is failure -> state is fault
   estimation_health.error.reset();
   estimation_health_subscriber.update_latest(estimation_health, current_ms);
   imu_health_subscriber.update_latest(imu_health, current_ms);          // keep providing good imu health status
   barometer_health_subscriber.update_latest(baro_health, current_ms);   // keep providing good baro health status
   current_ms++;
   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.imu_operational, true);
   EXPECT_EQ(health.estimation_operational, true);
   EXPECT_EQ(health.control_operational, true);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::fault);   // estimation failure -> flight critical
   EXPECT_EQ(health.control_health, aeromight_boundaries::SubsystemHealth::operational);

   // removing cause of fault
   estimation_health_subscriber.update_latest(estimation_health, current_ms);
   imu_health_subscriber.update_latest(imu_health, current_ms);           // keep providing good imu health status
   barometer_health_subscriber.update_latest(baro_health, current_ms);    // keep providing good baro health status
   control_health_subscriber.update_latest(control_health, current_ms);   // keep providing good control health status
   current_ms++;
   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.imu_operational, true);
   EXPECT_EQ(health.estimation_operational, true);
   EXPECT_EQ(health.control_operational, true);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);   // no longer flight critical
   EXPECT_EQ(health.control_health, aeromight_boundaries::SubsystemHealth::operational);
}

TEST_F(HealthMonitoringTest, control_failure_causes_flight_critical_fault)
{

   make_sensors_ready();

   make_estimation_and_control_ready();

   // good health for imu
   imu::ImuStatus imu_health{0, true};

   // good health for baro
   barometer::BarometerStatus baro_health{true};

   // good health for estimation and control at startup
   aeromight_boundaries::EstimatorStatus estimation_health{true, 0};
   aeromight_boundaries::ControlStatus   control_health{true, 0};

   aeromight_boundaries::HealthSummary health{};

   imu_health_subscriber.update_latest(imu_health, current_ms);                 // keep providing good imu health status
   barometer_health_subscriber.update_latest(baro_health, current_ms);          // keep providing good baro health status
   estimation_health_subscriber.update_latest(estimation_health, current_ms);   // keep providing good estimation health status
   control_health_subscriber.update_latest(control_health, current_ms);         // keep providing good control health status
   current_ms++;
   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.imu_operational, true);
   EXPECT_EQ(health.estimation_operational, true);
   EXPECT_EQ(health.control_operational, true);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.control_health, aeromight_boundaries::SubsystemHealth::operational);

   // control has now error
   control_health.error.set(static_cast<types::ErrorBitsType>(aeromight_boundaries::ControlStatus::Error::timing_jitter));
   imu_health_subscriber.update_latest(imu_health, current_ms);                 // keep providing good imu health status
   barometer_health_subscriber.update_latest(baro_health, current_ms);          // keep providing good baro health status
   estimation_health_subscriber.update_latest(estimation_health, current_ms);   // keep providing good estimation health status
   control_health_subscriber.update_latest(control_health, current_ms);
   current_ms++;
   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.imu_operational, true);
   EXPECT_EQ(health.estimation_operational, true);
   EXPECT_EQ(health.control_operational, true);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.control_health, aeromight_boundaries::SubsystemHealth::fault);   // control failure -> flight critical

   // removing cause of fault
   control_health.error = 0;
   imu_health_subscriber.update_latest(imu_health, current_ms);                 // keep providing good imu health status
   barometer_health_subscriber.update_latest(baro_health, current_ms);          // keep providing good baro health status
   estimation_health_subscriber.update_latest(estimation_health, current_ms);   // keep providing good estimation health status
   control_health_subscriber.update_latest(control_health, current_ms);
   current_ms++;
   sys_clock.m_sec = current_ms;
   health_monitoring.run_once();

   health = health_monitoring.get_health_summary();
   EXPECT_EQ(health.timestamp_ms, current_ms);
   EXPECT_EQ(health.imu_operational, true);
   EXPECT_EQ(health.estimation_operational, true);
   EXPECT_EQ(health.control_operational, true);
   EXPECT_EQ(health.imu_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.barometer_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.estimation_health, aeromight_boundaries::SubsystemHealth::operational);
   EXPECT_EQ(health.control_health, aeromight_boundaries::SubsystemHealth::operational);
}

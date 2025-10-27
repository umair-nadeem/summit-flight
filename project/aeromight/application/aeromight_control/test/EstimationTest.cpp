#include "aeromight_control/Estimation.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "aeromight_control/EkfState.hpp"
#include "mocks/common/ClockSource.hpp"
#include "mocks/common/Logger.hpp"

class AhrsFilterMock
{
public:
   MOCK_METHOD(void, update_in_ned_frame, (const math::Vector3&, const math::Vector3&, const float&), ());
   MOCK_METHOD(void, reset, ());
   MOCK_METHOD(math::Quaternion, get_quaternion, (), (const));
   MOCK_METHOD(math::Vector3, get_unbiased_gyro_data, (const math::Vector3&), ());
   MOCK_METHOD(math::Vector3, get_gyro_bias, (), (const));
};

class EkfMock
{
public:
   MOCK_METHOD(void, predict, (const math::Vector3&, const math::Quaternion&, const float), ());
   MOCK_METHOD(void, update, (const float), ());
   MOCK_METHOD(void, reset, ());
   MOCK_METHOD(aeromight_control::EkfState, get_ekf_state, (), (const));
};

class EstimationTest : public testing::Test
{
protected:
   void provide_ticks(const uint32_t n)
   {
      const uint32_t ms_count_after_ticks = n + current_ms;
      for (; current_ms < ms_count_after_ticks; current_ms++)
      {
         estimation.execute();
      }
   }

   void prepare_imu_sample(const std::optional<math::Vector3> accel_mps2, std::optional<math::Vector3> gyro_radps, const uint32_t timestamp_ms)
   {
      imu_sensor::ImuData data{accel_mps2, gyro_radps, std::nullopt};
      imu_data.update_latest(data, timestamp_ms);
   }

   void prepare_baro_sample(const std::optional<float> pressure_pa, const uint32_t timestamp_ms)
   {
      barometer_sensor::BarometerData data{pressure_pa, std::nullopt};
      baro_data.update_latest(data, timestamp_ms);
   }

   void provide_valid_reference_pressure()
   {
      const uint32_t ms_count_after_ticks = num_samples_reference_pressure + current_ms;
      for (; current_ms < ms_count_after_ticks; current_ms++)
      {
         float pressure;
         if ((current_ms & 1u) == false)
         {
            pressure = bmp390::params::max_plauisble_range_pressure_pa;
         }
         else
         {
            pressure = bmp390::params::min_plauisble_range_pressure_pa;
         }

         prepare_baro_sample(pressure, current_ms);
         estimation.execute();
      }

      EXPECT_EQ(estimation.get_state(), aeromight_boundaries::EstimatorState::running);
   }

   static constexpr uint8_t  max_recovery_attempts                          = 3u;
   static constexpr uint8_t  num_samples_reference_pressure                 = 3u;
   static constexpr uint32_t execution_period_ms                            = 4u;
   static constexpr uint32_t wait_timeout_reference_pressure_acquisition_ms = 40;
   static constexpr uint32_t max_age_imu_data_ms                            = 8u;
   static constexpr uint32_t max_age_baro_data_ms                           = 16u;
   static constexpr float    max_valid_imu_sample_dt_s                      = 0.01f;

   testing::NiceMock<AhrsFilterMock>                               ahrs_filter_mock{};
   testing::NiceMock<EkfMock>                                      ekf_mock{};
   mocks::common::ClockSource                                      sys_clock{};
   ::boundaries::SharedData<aeromight_boundaries::EstimatorHealth> estimator_health_storage{};
   aeromight_control::StateEstimation                              state_estimation_storage{};
   ::boundaries::SharedData<imu_sensor::ImuData>                   imu_data{};
   ::boundaries::SharedData<barometer_sensor::BarometerData>       baro_data{};
   mocks::common::Logger                                           logger{"estmation"};
   uint32_t                                                        current_ms{1u};

   aeromight_control::Estimation<decltype(ahrs_filter_mock),
                                 decltype(ekf_mock),
                                 decltype(sys_clock),
                                 decltype(logger)>
       estimation{ahrs_filter_mock,
                  ekf_mock,
                  estimator_health_storage,
                  state_estimation_storage,
                  imu_data,
                  baro_data,
                  logger,
                  max_recovery_attempts,
                  num_samples_reference_pressure,
                  execution_period_ms,
                  wait_timeout_reference_pressure_acquisition_ms,
                  max_age_imu_data_ms,
                  max_age_baro_data_ms,
                  max_valid_imu_sample_dt_s};
};

TEST_F(EstimationTest, start_and_stop)
{
   estimation.start();
   EXPECT_EQ(estimation.get_state(), aeromight_boundaries::EstimatorState::get_reference_pressure);

   estimation.stop();
   EXPECT_EQ(estimation.get_state(), aeromight_boundaries::EstimatorState::idle);
}

TEST_F(EstimationTest, fault_due_to_reference_pressure_acquisition_timeout)
{
   estimation.start();
   provide_ticks(wait_timeout_reference_pressure_acquisition_ms / execution_period_ms);

   aeromight_boundaries::EstimatorHealth::ErrorBits error{};
   error.set(static_cast<uint8_t>(aeromight_boundaries::EstimatorHealth::Error::reference_pressure_estimate_timeout));

   const aeromight_boundaries::EstimatorHealth health = estimator_health_storage.get_latest().data;
   EXPECT_EQ(health.error.to_ulong(), error.to_ulong());
   EXPECT_EQ(estimation.get_state(), aeromight_boundaries::EstimatorState::fault);
}

TEST_F(EstimationTest, fault_due_to_pressure_sample_with_missing_data)
{
   estimation.start();

   const uint32_t ticks_to_execute = (wait_timeout_reference_pressure_acquisition_ms / execution_period_ms) + current_ms;
   for (; current_ms < ticks_to_execute; current_ms++)
   {
      prepare_baro_sample(std::nullopt, current_ms);
      estimation.execute();
   }

   aeromight_boundaries::EstimatorHealth::ErrorBits error{};
   error.set(static_cast<uint8_t>(aeromight_boundaries::EstimatorHealth::Error::reference_pressure_estimate_timeout));
   error.set(static_cast<uint8_t>(aeromight_boundaries::EstimatorHealth::Error::missing_valid_baro_data));

   const aeromight_boundaries::EstimatorHealth health = estimator_health_storage.get_latest().data;
   EXPECT_EQ(health.error.to_ulong(), error.to_ulong());
   EXPECT_EQ(estimation.get_state(), aeromight_boundaries::EstimatorState::fault);
}

TEST_F(EstimationTest, fault_due_to_reference_pressure_acquisition_timeout_with_only_one_valid_sample_acquired)
{
   estimation.start();
   prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms++);

   const uint32_t ticks_to_execute = (wait_timeout_reference_pressure_acquisition_ms / execution_period_ms) + current_ms;
   for (; current_ms < ticks_to_execute; current_ms++)
   {
      estimation.execute();
   }

   aeromight_boundaries::EstimatorHealth::ErrorBits error{};
   error.set(static_cast<uint8_t>(aeromight_boundaries::EstimatorHealth::Error::reference_pressure_estimate_timeout));

   const aeromight_boundaries::EstimatorHealth health = estimator_health_storage.get_latest().data;
   EXPECT_EQ(health.error.to_ulong(), error.to_ulong());
   EXPECT_EQ(estimation.get_state(), aeromight_boundaries::EstimatorState::fault);
}

TEST_F(EstimationTest, fault_due_to_reference_pressure_below_plausible_range)
{
   estimation.start();

   const uint32_t ticks_to_execute = num_samples_reference_pressure + current_ms;
   for (; current_ms < ticks_to_execute; current_ms++)
   {
      prepare_baro_sample(bmp390::params::min_plauisble_range_pressure_pa - 1.0f, current_ms);
      estimation.execute();
   }

   aeromight_boundaries::EstimatorHealth::ErrorBits error{};
   error.set(static_cast<uint8_t>(aeromight_boundaries::EstimatorHealth::Error::reference_pressure_implausible));

   const aeromight_boundaries::EstimatorHealth health = estimator_health_storage.get_latest().data;
   EXPECT_EQ(health.error.to_ulong(), error.to_ulong());
   EXPECT_EQ(estimation.get_state(), aeromight_boundaries::EstimatorState::fault);
   EXPECT_EQ(estimation.get_reference_pressure(), bmp390::params::min_plauisble_range_pressure_pa - 1.0f);
}

TEST_F(EstimationTest, fault_due_to_reference_pressure_above_plausible_range)
{
   estimation.start();

   const uint32_t ticks_to_execute = num_samples_reference_pressure + current_ms;
   for (; current_ms < ticks_to_execute; current_ms++)
   {
      prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa + 1.0f, current_ms);
      estimation.execute();
   }

   aeromight_boundaries::EstimatorHealth::ErrorBits error{};
   error.set(static_cast<uint8_t>(aeromight_boundaries::EstimatorHealth::Error::reference_pressure_implausible));

   const aeromight_boundaries::EstimatorHealth health = estimator_health_storage.get_latest().data;
   EXPECT_EQ(health.error.to_ulong(), error.to_ulong());
   EXPECT_EQ(estimation.get_state(), aeromight_boundaries::EstimatorState::fault);
   EXPECT_EQ(estimation.get_reference_pressure(), bmp390::params::max_plauisble_range_pressure_pa + 1.0f);
}

TEST_F(EstimationTest, valid_reference_pressure)
{
   estimation.start();

   provide_valid_reference_pressure();

   const aeromight_boundaries::EstimatorHealth health = estimator_health_storage.get_latest().data;
   EXPECT_TRUE(health.valid_reference_pressure_acquired);
   EXPECT_EQ(health.error.to_ulong(), 0);
   EXPECT_EQ(estimation.get_state(), aeromight_boundaries::EstimatorState::running);
}

TEST_F(EstimationTest, filters_reset_due_to_large_time_gap_in_imu_samples)
{
   estimation.start();

   provide_valid_reference_pressure();

   // provide one valid imu and baro sample
   prepare_imu_sample(math::Vector3{}, math::Vector3{}, current_ms);
   prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms);
   sys_clock.m_sec = current_ms++;
   estimation.execute();

   EXPECT_CALL(ahrs_filter_mock, reset());
   EXPECT_CALL(ekf_mock, reset());

   current_ms = static_cast<uint32_t>(max_valid_imu_sample_dt_s * 1000.0f) + current_ms;
   prepare_imu_sample(math::Vector3{}, math::Vector3{}, current_ms);
   sys_clock.m_sec = current_ms;
   estimation.execute();

   EXPECT_EQ(estimation.get_state(), aeromight_boundaries::EstimatorState::running);
}

TEST_F(EstimationTest, run_attitude_estimation)
{
   estimation.start();

   provide_valid_reference_pressure();

   // provide one valid imu sample
   prepare_imu_sample(math::Vector3{}, math::Vector3{}, current_ms);
   sys_clock.m_sec = current_ms++;
   estimation.execute();

   const auto  accel_enu     = math::Vector3{1, 2, 3};
   const auto  gyro_enu      = math::Vector3{4, 5, 6};
   const auto  accel_ned     = math::Vector3{-2, -1, -3};
   const auto  gyro_ned      = math::Vector3{-5, -4, -6};
   const auto  unbiased_gyro = math::Vector3{6, 45, -6};
   const auto  quat          = math::Quaternion{1, 0, 9, 17};
   const float dt_s          = static_cast<float>(current_ms - (current_ms - 1u)) * 0.001f;

   EXPECT_CALL(ahrs_filter_mock, update_in_ned_frame(accel_ned, gyro_ned, dt_s));
   EXPECT_CALL(ahrs_filter_mock, get_quaternion()).WillOnce(::testing::Return(quat));
   EXPECT_CALL(ahrs_filter_mock, get_unbiased_gyro_data(gyro_ned)).WillOnce(::testing::Return(unbiased_gyro));
   EXPECT_CALL(ekf_mock, predict(accel_ned, quat, dt_s));

   prepare_imu_sample(accel_enu, gyro_enu, current_ms);
   sys_clock.m_sec = current_ms;
   estimation.execute();

   EXPECT_EQ(estimation.get_state(), aeromight_boundaries::EstimatorState::running);
   EXPECT_EQ(state_estimation_storage.attitude, quat);
   EXPECT_EQ(state_estimation_storage.gyro_radps, unbiased_gyro);
   EXPECT_EQ(state_estimation_storage.altitude, 0.0f);            // altitude estimation hasn't run
   EXPECT_EQ(state_estimation_storage.vertical_velocity, 0.0f);   // altitude estimation hasn't run
   EXPECT_EQ(state_estimation_storage.timestamp_ms, current_ms);
}

TEST_F(EstimationTest, run_altitude_estimation)
{
   estimation.start();

   provide_valid_reference_pressure();

   // provide one valid baro sample
   prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms);
   sys_clock.m_sec = current_ms++;
   estimation.execute();

   const float                       altitude = utilities::Barometric::convert_pressure_to_altitude(bmp390::params::max_plauisble_range_pressure_pa);
   const aeromight_control::EkfState ekf_state{10.0f, 15.0f, 20.0f};

   EXPECT_CALL(ekf_mock, update(altitude));
   EXPECT_CALL(ekf_mock, get_ekf_state()).WillOnce(testing::Return(ekf_state));

   prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms);
   sys_clock.m_sec = current_ms;
   estimation.execute();

   EXPECT_EQ(estimation.get_state(), aeromight_boundaries::EstimatorState::running);
   EXPECT_EQ(state_estimation_storage.attitude, math::Quaternion{});   // attitude estimation hasn't run
   EXPECT_EQ(state_estimation_storage.gyro_radps, math::Vector3{});    // attitude estimation hasn't run
   EXPECT_NEAR(state_estimation_storage.altitude, 10.0f, 0.001f);
   EXPECT_NEAR(state_estimation_storage.vertical_velocity, 15.0f, 0.001f);
   EXPECT_EQ(state_estimation_storage.timestamp_ms, current_ms);
}

TEST_F(EstimationTest, fault_due_to_stale_imu_data)
{
   estimation.start();

   provide_valid_reference_pressure();

   // provide normal sample for both sensors once
   prepare_imu_sample(math::Vector3{}, math::Vector3{}, current_ms);
   prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms);
   sys_clock.m_sec = current_ms++;
   estimation.execute();

   const uint32_t ticks_to_execute = max_age_imu_data_ms + current_ms + 1u;
   for (; current_ms < ticks_to_execute; current_ms++)
   {
      sys_clock.m_sec = current_ms;
      prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms);   // keep providing valid baro data
      estimation.execute();
   }

   aeromight_boundaries::EstimatorHealth::ErrorBits error{};
   error.set(static_cast<uint8_t>(aeromight_boundaries::EstimatorHealth::Error::stale_imu_sensor_data));

   const aeromight_boundaries::EstimatorHealth health = estimator_health_storage.get_latest().data;
   EXPECT_TRUE(health.valid_reference_pressure_acquired);
   EXPECT_EQ(health.error.to_ulong(), error.to_ulong());
   EXPECT_EQ(estimation.get_state(), aeromight_boundaries::EstimatorState::fault);
}

TEST_F(EstimationTest, fault_due_to_stale_baro_data)
{
   estimation.start();

   provide_valid_reference_pressure();

   // provide normal sample for both sensors once
   prepare_imu_sample(math::Vector3{}, math::Vector3{}, current_ms);
   prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms);
   sys_clock.m_sec = current_ms++;
   estimation.execute();

   const uint32_t ticks_to_execute = max_age_baro_data_ms + current_ms + 1u;
   for (; current_ms < ticks_to_execute; current_ms++)
   {
      sys_clock.m_sec = current_ms;
      prepare_imu_sample(math::Vector3{}, math::Vector3{}, current_ms);   // keep providing valid imu data
      estimation.execute();
   }

   aeromight_boundaries::EstimatorHealth::ErrorBits error{};
   error.set(static_cast<uint8_t>(aeromight_boundaries::EstimatorHealth::Error::stale_baro_sensor_data));

   const aeromight_boundaries::EstimatorHealth health = estimator_health_storage.get_latest().data;
   EXPECT_TRUE(health.valid_reference_pressure_acquired);
   EXPECT_EQ(health.error.to_ulong(), error.to_ulong());
   EXPECT_EQ(estimation.get_state(), aeromight_boundaries::EstimatorState::fault);
}

TEST_F(EstimationTest, fault_recovery_before_reference_pressure_acquisition)
{
   estimation.start();

   aeromight_boundaries::EstimatorHealth health{};
   for (uint32_t i = 0; i < max_recovery_attempts; i++)
   {
      provide_ticks(wait_timeout_reference_pressure_acquisition_ms / execution_period_ms);
      estimation.execute();   // attempt recovery
      health = estimator_health_storage.get_latest().data;
      EXPECT_EQ(health.recovery_attempts, i + 1u);
   }

   provide_ticks(wait_timeout_reference_pressure_acquisition_ms / execution_period_ms);
   estimation.execute();   // don't attempt recovery
   health = estimator_health_storage.get_latest().data;
   EXPECT_EQ(health.recovery_attempts, 3u);
}

TEST_F(EstimationTest, fault_recovery_after_reference_pressure_acquisition)
{
   estimation.start();

   provide_valid_reference_pressure();

   // provide normal sample for both sensors once
   prepare_imu_sample(math::Vector3{}, math::Vector3{}, current_ms);
   prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms);
   sys_clock.m_sec = current_ms++;
   estimation.execute();

   aeromight_boundaries::EstimatorHealth health = estimator_health_storage.get_latest().data;
   EXPECT_TRUE(health.valid_reference_pressure_acquired);
   EXPECT_EQ(health.error.to_ulong(), 0);

   for (uint32_t i = 0; i < max_recovery_attempts; i++)
   {
      current_ms += max_age_imu_data_ms;
      sys_clock.m_sec = current_ms;
      estimation.execute();
      estimation.execute();   // attempt recovery
      health = estimator_health_storage.get_latest().data;
      EXPECT_EQ(health.recovery_attempts, i + 1u);
   }

   current_ms += max_age_imu_data_ms;
   sys_clock.m_sec = current_ms;
   estimation.execute();
   estimation.execute();   // don't attempt recovery
   health = estimator_health_storage.get_latest().data;
   EXPECT_EQ(health.recovery_attempts, 3u);
}

TEST_F(EstimationTest, successful_recovery_after_reference_pressure_acquisition)
{
   estimation.start();

   provide_valid_reference_pressure();

   // provide normal sample for both sensors once
   prepare_imu_sample(math::Vector3{}, math::Vector3{}, current_ms);
   prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms);
   sys_clock.m_sec = current_ms++;
   estimation.execute();

   aeromight_boundaries::EstimatorHealth health = estimator_health_storage.get_latest().data;
   EXPECT_TRUE(health.valid_reference_pressure_acquired);
   EXPECT_EQ(health.error.to_ulong(), 0);

   for (uint32_t i = 0; i < max_recovery_attempts; i++)
   {
      current_ms += max_age_imu_data_ms;
      sys_clock.m_sec = current_ms;
      estimation.execute();
      estimation.execute();   // attempt recovery
      health = estimator_health_storage.get_latest().data;
      EXPECT_EQ(health.recovery_attempts, i + 1u);
   }

   // recover successfully with data
   const float                       altitude = utilities::Barometric::convert_pressure_to_altitude(bmp390::params::max_plauisble_range_pressure_pa);
   const aeromight_control::EkfState ekf_state{10.0f, 15.0f, 20.0f};

   EXPECT_CALL(ekf_mock, update(altitude));
   EXPECT_CALL(ekf_mock, get_ekf_state()).WillOnce(testing::Return(ekf_state));

   prepare_imu_sample(math::Vector3{}, math::Vector3{}, current_ms);
   prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms);
   sys_clock.m_sec = current_ms;
   estimation.execute();

   health = estimator_health_storage.get_latest().data;
   EXPECT_EQ(health.recovery_attempts, 3u);
   EXPECT_EQ(health.state, aeromight_boundaries::EstimatorState::running);

   EXPECT_EQ(state_estimation_storage.attitude, math::Quaternion{});   // attitude estimation hasn't run
   EXPECT_EQ(state_estimation_storage.gyro_radps, math::Vector3{});    // attitude estimation hasn't run
   EXPECT_NEAR(state_estimation_storage.altitude, 10.0f, 0.001f);
   EXPECT_NEAR(state_estimation_storage.vertical_velocity, 15.0f, 0.001f);
   EXPECT_EQ(state_estimation_storage.timestamp_ms, current_ms);
}

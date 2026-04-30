#include "aeromight_estimation/Estimation.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "aeromight_estimation/EkfState.hpp"
#include "bmp390/params.hpp"
#include "logging/Logger.hpp"
#include "mocks/sys_time/ClockSource.hpp"
#include "utilities/Barometric.hpp"

class AttitudeEstimatorMock
{
public:
   MOCK_METHOD(void, update, (const math::Vec3f&, const math::Vec3f&, const float&), ());
   MOCK_METHOD(void, reset, ());
   MOCK_METHOD(math::Quaternion, get_quaternion, (), (const));
   MOCK_METHOD(math::Vec3f, get_unbiased_gyro_data, (const math::Vec3f&), ());
   MOCK_METHOD(math::Vec3f, get_gyro_bias, (), (const));
};

class EkfMock
{
public:
   MOCK_METHOD(void, predict, (const math::Vec3f&, const math::Quaternion&, const float), ());
   MOCK_METHOD(void, update, (const float), ());
   MOCK_METHOD(void, reset, ());
   MOCK_METHOD(aeromight_estimation::EkfState, get_ekf_state, (), (const));
};

class EstimationTest : public testing::Test
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
         estimation.execute();
      }
   }

   void prepare_imu_sample(const std::optional<math::Vec3f> accel_mps2, std::optional<math::Vec3f> gyro_radps, const uint32_t timestamp_ms)
   {
      imu::ImuData data{accel_mps2, gyro_radps, std::nullopt};
      imu_data.update_latest(data, timestamp_ms);
   }

   void prepare_baro_sample(const std::optional<float> pressure_pa, const uint32_t timestamp_ms, const float altitude_m = 0.0f)
   {
      barometer::BarometerData data{pressure_pa, altitude_m, std::nullopt};
      baro_data.update_latest(data, timestamp_ms);
   }

   aeromight_estimation::EstimationParams params{
       .run_altitude_estimation         = true,
       .max_age_imu_data_ms             = 8u,
       .max_age_baro_data_ms            = 16u,
       .max_valid_imu_sample_dt_s       = 0.01f,
       .max_valid_barometer_sample_dt_s = 2.0f};

   testing::NiceMock<AttitudeEstimatorMock>                      ahrs_filter_mock{};
   testing::NiceMock<EkfMock>                                    ekf_mock{};
   mocks::sys_time::ClockSource                                  sys_clock{};
   boundaries::SharedData<aeromight_boundaries::EstimatorStatus> estimator_status{};
   aeromight_boundaries::StateEstimation                         state_estimation_storage{};
   boundaries::SharedData<imu::ImuData>                          imu_data{};
   boundaries::SharedData<barometer::BarometerData>              baro_data{};
   logging::Logger                                               logger{"estmation"};
   uint32_t                                                      current_ms{0};

   aeromight_estimation::Estimation<decltype(ahrs_filter_mock),
                                    decltype(ekf_mock),
                                    decltype(sys_clock),
                                    decltype(logger)>
       estimation{ahrs_filter_mock,
                  ekf_mock,
                  estimator_status,
                  state_estimation_storage,
                  imu_data,
                  baro_data,
                  logger,
                  params};
};

TEST_F(EstimationTest, filters_reset_due_to_large_time_gap_in_imu_samples)
{
   estimation.start();

   // provide one valid imu and baro sample
   prepare_imu_sample(math::Vec3f{}, math::Vec3f{}, current_ms);
   prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms);
   sys_clock.m_sec = current_ms++;
   estimation.execute();

   const uint32_t ticks_needed = static_cast<uint32_t>(params.max_valid_imu_sample_dt_s * 1000.0f);
   for (uint32_t i = 0; i < ticks_needed; i++)
   {
      prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms);   // normal update for baro
      sys_clock.m_sec = current_ms++;
      estimation.execute();
   }

   EXPECT_CALL(ahrs_filter_mock, reset());
   EXPECT_CALL(ekf_mock, reset());

   prepare_imu_sample(math::Vec3f{}, math::Vec3f{}, current_ms);
   sys_clock.m_sec = current_ms++;
   estimation.execute();
}

TEST_F(EstimationTest, filters_reset_due_to_large_time_gap_in_barometer_samples)
{
   estimation.start();

   // provide one valid imu and baro sample
   prepare_imu_sample(math::Vec3f{}, math::Vec3f{}, current_ms);
   prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms);
   sys_clock.m_sec = current_ms++;
   estimation.execute();

   const uint32_t ticks_needed = static_cast<uint32_t>(params.max_valid_barometer_sample_dt_s * 1000.0f);
   for (uint32_t i = 0; i < ticks_needed; i++)
   {
      prepare_imu_sample(math::Vec3f{}, math::Vec3f{}, current_ms);   // normal update for imu
      sys_clock.m_sec = current_ms++;
      estimation.execute();
   }

   EXPECT_CALL(ekf_mock, reset());

   prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms);
   sys_clock.m_sec = current_ms++;
   estimation.execute();
}

TEST_F(EstimationTest, run_attitude_estimation)
{
   estimation.start();

   // provide one valid imu sample
   prepare_imu_sample(math::Vec3f{}, math::Vec3f{}, current_ms);
   sys_clock.m_sec = current_ms++;
   estimation.execute();

   const auto  accel_enu     = math::Vec3f{1, 2, 3};
   const auto  gyro_enu      = math::Vec3f{4, 5, 6};
   const auto  accel_ned     = math::Vec3f{-2, -1, -3};
   const auto  gyro_ned      = math::Vec3f{-5, -4, -6};
   const auto  unbiased_gyro = math::Vec3f{6, 45, -6};
   const auto  quat          = math::Quaternion{1, 0, 9, 17};
   const float dt_s          = static_cast<float>(current_ms - (current_ms - 1u)) * 0.001f;

   EXPECT_CALL(ahrs_filter_mock, update(accel_ned, gyro_ned, dt_s));
   EXPECT_CALL(ahrs_filter_mock, get_quaternion()).WillOnce(::testing::Return(quat));
   EXPECT_CALL(ahrs_filter_mock, get_unbiased_gyro_data(gyro_ned)).WillOnce(::testing::Return(unbiased_gyro));
   EXPECT_CALL(ekf_mock, predict(accel_ned, quat, dt_s));

   prepare_imu_sample(accel_enu, gyro_enu, current_ms);
   sys_clock.m_sec = current_ms;
   estimation.execute();

   EXPECT_EQ(state_estimation_storage.raw_gyro_radps, unbiased_gyro);
   EXPECT_EQ(state_estimation_storage.altitude, 0.0f);            // altitude estimation hasn't run
   EXPECT_EQ(state_estimation_storage.vertical_velocity, 0.0f);   // altitude estimation hasn't run
   EXPECT_EQ(state_estimation_storage.timestamp_ms, current_ms);
}

TEST_F(EstimationTest, run_altitude_estimation)
{
   estimation.start();

   // provide one valid baro sample
   prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms);
   sys_clock.m_sec = current_ms++;
   estimation.execute();

   const float                          altitude = utilities::Barometric::convert_pressure_to_altitude(bmp390::params::max_plauisble_range_pressure_pa);
   const aeromight_estimation::EkfState ekf_state{10.0f, 15.0f, 20.0f};

   EXPECT_CALL(ekf_mock, update(altitude));
   EXPECT_CALL(ekf_mock, get_ekf_state()).WillOnce(testing::Return(ekf_state));

   prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms);
   sys_clock.m_sec = current_ms;
   estimation.execute();

   EXPECT_EQ(state_estimation_storage.raw_gyro_radps, math::Vec3f{});   // attitude estimation hasn't run
   EXPECT_NEAR(state_estimation_storage.altitude, 10.0f, 0.001f);
   EXPECT_NEAR(state_estimation_storage.vertical_velocity, 15.0f, 0.001f);
   EXPECT_EQ(state_estimation_storage.timestamp_ms, current_ms);
}

TEST_F(EstimationTest, fault_due_to_stale_imu_data)
{
   estimation.start();

   // provide normal sample for both sensors once
   prepare_imu_sample(math::Vec3f{}, math::Vec3f{}, current_ms);
   prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms);
   sys_clock.m_sec = current_ms++;
   estimation.execute();

   const uint32_t ticks_to_execute = params.max_age_imu_data_ms + current_ms + 1u;
   for (; current_ms < ticks_to_execute; current_ms++)
   {
      sys_clock.m_sec = current_ms;
      prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms);   // keep providing valid baro data
      estimation.execute();
   }

   aeromight_boundaries::EstimatorStatus::ErrorBits error{};
   error.set(static_cast<types::ErrorBitsType>(aeromight_boundaries::EstimatorStatus::Error::stale_imu_sensor_data));

   const aeromight_boundaries::EstimatorStatus health = estimator_status.get_latest().data;
   EXPECT_EQ(health.error.to_ulong(), error.to_ulong());
}

TEST_F(EstimationTest, no_fault_due_to_stale_baro_data)
{
   estimation.start();

   // provide normal sample for both sensors once
   prepare_imu_sample(math::Vec3f{}, math::Vec3f{}, current_ms);
   prepare_baro_sample(bmp390::params::max_plauisble_range_pressure_pa, current_ms);
   sys_clock.m_sec = current_ms++;
   estimation.execute();

   const uint32_t ticks_to_execute = params.max_age_baro_data_ms + current_ms + 1u;
   for (; current_ms < ticks_to_execute; current_ms++)
   {
      sys_clock.m_sec = current_ms;
      prepare_imu_sample(math::Vec3f{}, math::Vec3f{}, current_ms);   // keep providing valid imu data
      estimation.execute();
   }

   aeromight_boundaries::EstimatorStatus::ErrorBits error{};
   error.set(static_cast<types::ErrorBitsType>(aeromight_boundaries::EstimatorStatus::Error::stale_baro_sensor_data));

   const aeromight_boundaries::EstimatorStatus health = estimator_status.get_latest().data;
   EXPECT_EQ(health.error.to_ulong(), error.to_ulong());
}

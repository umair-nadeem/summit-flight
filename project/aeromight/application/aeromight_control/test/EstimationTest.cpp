#include "aeromight_control/Estimation.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "mocks/common/Logger.hpp"
#include "sys_time/ClockSource.hpp"

class AhrsFilterMock
{
public:
   MOCK_METHOD(void, update, (const math::Vector3&, const math::Vector3&, const float&), ());
   MOCK_METHOD(void, reset, ());
};

class EkfMock
{
public:
   MOCK_METHOD(void, update, (const math::Vector3&, const math::Vector3&), ());
   MOCK_METHOD(void, reset, ());
};

class EstimationTest : public testing::Test
{
protected:
   static constexpr uint8_t     max_recovery_attempts                          = 3u;
   static constexpr uint8_t     num_samples_pressure_reference                 = 3u;
   static constexpr std::size_t execution_period_ms                            = 4u;
   static constexpr std::size_t wait_timeout_pressure_reference_acquisition_ms = 40;
   static constexpr std::size_t max_age_imu_data_ms                            = 8u;
   static constexpr std::size_t max_age_baro_data_ms                           = 16u;
   static constexpr float       max_valid_imu_sample_dt_s                      = 0.02f;

   AhrsFilterMock                                                  ahrs_filter_mock{};
   EkfMock                                                         ekf_mock{};
   ::boundaries::SharedData<aeromight_boundaries::EstimatorHealth> estimator_health_storage{};
   aeromight_control::StateEstimation                              state_estimation_storage{};
   ::boundaries::SharedData<imu_sensor::ImuData>                   imu_data{};
   ::boundaries::SharedData<barometer_sensor::BarometerData>       baro_data{};
   mocks::common::Logger                                           logger{"estmation"};

   aeromight_control::Estimation<decltype(ahrs_filter_mock),
                                 decltype(ekf_mock),
                                 sys_time::ClockSource,
                                 decltype(logger)>
       estimation{ahrs_filter_mock,
                  ekf_mock,
                  estimator_health_storage,
                  state_estimation_storage,
                  imu_data,
                  baro_data,
                  logger,
                  max_recovery_attempts,
                  num_samples_pressure_reference,
                  execution_period_ms,
                  wait_timeout_pressure_reference_acquisition_ms,
                  max_age_imu_data_ms,
                  max_age_baro_data_ms,
                  max_valid_imu_sample_dt_s};
};

TEST_F(EstimationTest, check_start_and_stop)
{
   estimation.start();
}

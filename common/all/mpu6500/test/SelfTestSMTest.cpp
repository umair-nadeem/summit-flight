#include "mpu6500/SelfTestSM.hpp"

#include "Mpu6500BaseTest.hpp"
#include "mpu6500/Mpu6500StateHandler.hpp"

class SelfTestSMTest : public Mpu6500BaseTest
{
protected:
   using StateHandler    = mpu6500::Mpu6500StateHandler<sys_time::ClockSource, decltype(spi_master_with_dma), mocks::common::Logger>;
   using StateMachineDef = mpu6500::SelfTestStateMachine<StateHandler>;

   StateHandler mpu6500_handler{imu_data_storage,
                                imu_health_storage,
                                spi_master_with_dma,
                                logger,
                                read_failures_limit,
                                execution_period_ms,
                                receive_wait_timeout_ms,
                                sample_rate_divider,
                                dlpf_config,
                                gyro_full_scale,
                                accel_full_scale,
                                accel_a_dlpf_config,
                                gyro_range_plausibility_margin_radps,
                                accel_range_plausibility_margin_mps2,
                                num_samples_self_test,
                                gyro_tolerance_radps,
                                accel_tolerance_mps2};

   boost::sml::sm<StateMachineDef> sm{mpu6500_handler};
};

TEST_F(SelfTestSMTest, collect_samples_state)
{
   EXPECT_TRUE(sm.is(StateMachineDef::s_collect_samples));
}

TEST_F(SelfTestSMTest, read_data_timeout_causes_self_test_failure)
{
   sm.process_event(mpu6500::EventTick{});

   // cause receive wait timeout
   sm.process_event(mpu6500::EventTick{});
   sm.process_event(mpu6500::EventTick{});

   imu_sensor::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::bus_error));
   EXPECT_EQ(mpu6500_handler.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_FALSE(mpu6500_handler.self_test_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(SelfTestSMTest, all_zero_data_causes_self_test_failure)
{
   // set all zero data
   rx_buffer.fill(0);

   // trigger read data command
   sm.process_event(mpu6500::EventTick{});

   // cause receive wait timeout
   sm.process_event(mpu6500::EventReceiveDone{});

   imu_sensor::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::data_pattern_error));
   EXPECT_EQ(mpu6500_handler.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_FALSE(mpu6500_handler.self_test_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(SelfTestSMTest, all_ones_data_causes_self_test_failure)
{
   // set all ones data
   rx_buffer.fill(0xff);

   // trigger read data command
   sm.process_event(mpu6500::EventTick{});

   // cause receive wait timeout
   sm.process_event(mpu6500::EventReceiveDone{});

   imu_sensor::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::data_pattern_error));
   EXPECT_EQ(mpu6500_handler.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_FALSE(mpu6500_handler.self_test_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(SelfTestSMTest, implausible_temp_values_causes_self_test_failure)
{
   rx_buffer.fill(0);

   rx_buffer[7] = 0xc2;   // temperature->-26 degC
   rx_buffer[8] = 0xb4;
   sm.process_event(mpu6500::EventTick{});
   sm.process_event(mpu6500::EventReceiveDone{});

   rx_buffer[7] = 0x54;   // temperature->+86 degC
   rx_buffer[8] = 0xc6;
   sm.process_event(mpu6500::EventTick{});
   sm.process_event(mpu6500::EventReceiveDone{});

   rx_buffer[7] = 0xc2;   // temperature->+86 degC
   rx_buffer[8] = 0xb4;
   sm.process_event(mpu6500::EventTick{});
   sm.process_event(mpu6500::EventReceiveDone{});

   imu_sensor::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::out_of_range_data_error));
   EXPECT_EQ(mpu6500_handler.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_FALSE(mpu6500_handler.self_test_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(SelfTestSMTest, non_stationary_platform_causes_self_test_failure)
{
   rx_buffer.fill(0);

   provide_dynamic_accel_values();

   for (std::size_t i = 0; i < num_samples_self_test; i++)
   {
      sm.process_event(mpu6500::EventTick{});
      sm.process_event(mpu6500::EventReceiveDone{});
   }

   imu_sensor::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::non_stationary_calibration_error));
   EXPECT_EQ(mpu6500_handler.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_FALSE(mpu6500_handler.self_test_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(SelfTestSMTest, unstable_accel_values_cause_self_test_failure)
{
   rx_buffer.fill(0);

   const uint8_t deviation = 4u;   // cause variance in data
   for (std::size_t i = 0; i < num_samples_self_test; i++)
   {
      provide_stationary_accel_values(static_cast<uint8_t>(deviation * i));
      sm.process_event(mpu6500::EventTick{});
      sm.process_event(mpu6500::EventReceiveDone{});
   }

   imu_sensor::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::unstable_accel_error));
   EXPECT_EQ(mpu6500_handler.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_FALSE(mpu6500_handler.self_test_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(SelfTestSMTest, unstable_gyro_values_cause_self_test_failure)
{
   rx_buffer.fill(0);

   provide_stationary_accel_values();

   const uint8_t deviation = 4u;   // cause variance in data
   for (std::size_t i = 0; i < num_samples_self_test; i++)
   {
      provide_stationary_gyro_values(static_cast<uint8_t>(deviation * i));
      sm.process_event(mpu6500::EventTick{});
      sm.process_event(mpu6500::EventReceiveDone{});
   }

   imu_sensor::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::unstable_gyro_error));
   EXPECT_EQ(mpu6500_handler.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_FALSE(mpu6500_handler.self_test_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(SelfTestSMTest, sane_sensor_values_for_passing_self_test)
{
   rx_buffer.fill(0);

   provide_stationary_accel_values();
   provide_stationary_gyro_values();

   for (std::size_t i = 0; i < num_samples_self_test; i++)
   {
      sm.process_event(mpu6500::EventTick{});
      sm.process_event(mpu6500::EventReceiveDone{});
   }

   EXPECT_EQ(mpu6500_handler.get_error().to_ulong(), 0);
   EXPECT_TRUE(mpu6500_handler.self_test_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));

   const auto          bias       = mpu6500_handler.get_bias();
   const math::Vector3 accel_bias = std::get<0>(bias);
   const math::Vector3 gyro_bias  = std::get<1>(bias);

   EXPECT_NEAR(accel_bias[0], -1.01f, 0.01f);
   EXPECT_NEAR(accel_bias[1], 0.01f, 0.01f);
   EXPECT_NEAR(accel_bias[2], 0.70f, 0.01f);
   EXPECT_NEAR(gyro_bias[0], 0.01f, 0.01f);
   EXPECT_NEAR(gyro_bias[1], 0.01f, 0.01f);
   EXPECT_NEAR(gyro_bias[2], 0.01f, 0.01f);
}

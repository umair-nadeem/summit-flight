#include "mpu6500/ValidationSM.hpp"

#include "Mpu6500BaseTest.hpp"
#include "mpu6500/Mpu6500StateHandler.hpp"

class ValidationSMTest : public Mpu6500BaseTest
{
protected:
   using StateHandler    = mpu6500::Mpu6500StateHandler<sys_time::ClockSource, decltype(spi_master_with_dma), mocks::common::Logger>;
   using StateMachineDef = mpu6500::ValidationStateMachine<StateHandler>;

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

TEST_F(ValidationSMTest, check_diable_i2c_state)
{
   EXPECT_TRUE(sm.is(StateMachineDef::s_disable_i2c));
}

TEST_F(ValidationSMTest, check_diable_i2c_command)
{
   // trigger diable i2c command
   sm.process_event(mpu6500::EventTick{});

   test_buffer[0] = mpu6500::params::user_ctrl_reg;
   test_buffer[1] = 0x10;

   EXPECT_THAT(tx_buffer, testing::ElementsAreArray(test_buffer.begin(), test_buffer.end()));
   EXPECT_TRUE(sm.is(StateMachineDef::s_wakeup));
}

TEST_F(ValidationSMTest, check_clock_and_wakeup_command)
{
   sm.process_event(mpu6500::EventTick{});
   EXPECT_TRUE(sm.is(StateMachineDef::s_wakeup));

   // trigger clock and wakeup command
   sm.process_event(mpu6500::EventTick{});

   test_buffer[0] = mpu6500::params::pwr_mgmt_1_reg;
   test_buffer[1] = 0x01;

   EXPECT_THAT(tx_buffer, testing::ElementsAreArray(test_buffer.begin(), test_buffer.end()));
   EXPECT_TRUE(sm.is(StateMachineDef::s_read_id));
}

TEST_F(ValidationSMTest, check_read_id_wait_timeout)
{
   sm.process_event(mpu6500::EventTick{});
   sm.process_event(mpu6500::EventTick{});
   EXPECT_TRUE(sm.is(StateMachineDef::s_read_id));

   // trigger read id command
   sm.process_event(mpu6500::EventTick{});

   test_buffer[0] = 0x80 | mpu6500::params::who_am_i_reg;
   test_buffer[1] = 0x00;

   EXPECT_THAT(tx_buffer, testing::ElementsAreArray(test_buffer.begin(), test_buffer.end()));
   EXPECT_TRUE(sm.is(StateMachineDef::s_id_receive_wait));

   EXPECT_TRUE(mpu6500_handler.get_error().none());
   // passage of time will cause timeout
   sm.process_event(mpu6500::EventTick{});
   sm.process_event(mpu6500::EventTick{});

   EXPECT_TRUE(mpu6500_handler.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::bus_error)));
   EXPECT_FALSE(mpu6500_handler.validation_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(ValidationSMTest, check_read_id_mismatch)
{
   sm.process_event(mpu6500::EventTick{});
   sm.process_event(mpu6500::EventTick{});
   EXPECT_TRUE(sm.is(StateMachineDef::s_read_id));

   // trigger read id command
   rx_buffer[0] = 0x70;
   rx_buffer[1] = 0x71;

   sm.process_event(mpu6500::EventTick{});

   // send receive done event
   sm.process_event(mpu6500::EventReceiveDone{});

   EXPECT_TRUE(sm.is(StateMachineDef::s_verify_id));

   // id verification fails
   sm.process_event(mpu6500::EventTick{});

   imu_sensor::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::id_mismatch_error));

   EXPECT_EQ(mpu6500_handler.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_FALSE(mpu6500_handler.validation_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(ValidationSMTest, verify_id_successfully)
{
   sm.process_event(mpu6500::EventTick{});
   sm.process_event(mpu6500::EventTick{});
   EXPECT_TRUE(sm.is(StateMachineDef::s_read_id));

   // trigger read id command
   rx_buffer[0] = 0x00;
   rx_buffer[1] = 0x70;

   sm.process_event(mpu6500::EventTick{});

   // send receive done event
   sm.process_event(mpu6500::EventReceiveDone{});

   EXPECT_TRUE(sm.is(StateMachineDef::s_verify_id));

   // id verification fails
   sm.process_event(mpu6500::EventTick{});

   EXPECT_TRUE(mpu6500_handler.get_error().none());
   EXPECT_TRUE(mpu6500_handler.validation_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

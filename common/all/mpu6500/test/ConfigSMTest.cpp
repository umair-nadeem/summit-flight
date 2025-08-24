#include "mpu6500/ConfigSM.hpp"

#include "Mpu6500BaseTest.hpp"
#include "mpu6500/Mpu6500StateHandler.hpp"

class ConfigSMTest : public Mpu6500BaseTest
{
protected:
   using StateHandler    = mpu6500::Mpu6500StateHandler<sys_time::ClockSource, decltype(spi_master_with_dma)>;
   using StateMachineDef = mpu6500::ConfigStateMachine<StateHandler>;

   StateHandler                    mpu6500_handler{imu_data, imu_health, spi_master_with_dma, execution_period_ms, receive_wait_timeout_ms};
   boost::sml::sm<StateMachineDef> sm{mpu6500_handler};
};

TEST_F(ConfigSMTest, check_initial_config_state)
{
   EXPECT_TRUE(sm.is(StateMachineDef::s_config));
}

TEST_F(ConfigSMTest, check_burst_config)
{
   // trigger burst config
   sm.process_event(mpu6500::EventTick{});

   test_buffer[0] = mpu6500::params::smplrt_div_reg;
   test_buffer[1] = 0x03;
   test_buffer[2] = 0x02;
   test_buffer[3] = 0b0001'1000;
   test_buffer[4] = 0b0001'1000;
   test_buffer[5] = 0x03;

   EXPECT_THAT(tx_buffer, testing::ElementsAreArray(test_buffer.begin(), test_buffer.end()));
   EXPECT_TRUE(sm.is(StateMachineDef::s_read_config));
}

TEST_F(ConfigSMTest, check_write_burst_config_command)
{
   // trigger write burst config
   sm.process_event(mpu6500::EventTick{});

   test_buffer[0] = mpu6500::params::smplrt_div_reg;
   test_buffer[1] = 0x03;
   test_buffer[2] = 0x02;
   test_buffer[3] = 0b0001'1000;
   test_buffer[4] = 0b0001'1000;
   test_buffer[5] = 0x03;

   EXPECT_THAT(tx_buffer, testing::ElementsAreArray(test_buffer.begin(), test_buffer.end()));
   EXPECT_TRUE(sm.is(StateMachineDef::s_read_config));
}

TEST_F(ConfigSMTest, check_read_burst_config_command)
{
   // trigger write and read burst config commands
   sm.process_event(mpu6500::EventTick{});
   sm.process_event(mpu6500::EventTick{});

   test_buffer[0] = mpu6500::params::read_mask | mpu6500::params::smplrt_div_reg;
   test_buffer[1] = 0x00;
   test_buffer[2] = 0x00;
   test_buffer[3] = 0x00;
   test_buffer[4] = 0x00;
   test_buffer[5] = 0x00;

   EXPECT_THAT(tx_buffer, testing::ElementsAreArray(test_buffer.begin(), test_buffer.end()));
   EXPECT_TRUE(sm.is(StateMachineDef::s_config_receive_wait));
}

TEST_F(ConfigSMTest, check_read_burst_wait_timeout)
{
   // trigger write and read burst config commands
   sm.process_event(mpu6500::EventTick{});
   sm.process_event(mpu6500::EventTick{});

   EXPECT_TRUE(sm.is(StateMachineDef::s_config_receive_wait));

   // provide tick while in wait state
   sm.process_event(mpu6500::EventTick{});

   // still in receive wait
   EXPECT_TRUE(sm.is(StateMachineDef::s_config_receive_wait));

   sm.process_event(mpu6500::EventTick{});

   EXPECT_EQ(mpu6500_handler.get_error(), imu_sensor::ImuSensorError::bus_error);
   EXPECT_FALSE(mpu6500_handler.config_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(ConfigSMTest, check_failed_config_due_to_smplrt_mismatch)
{
   // trigger write burst config command
   sm.process_event(mpu6500::EventTick{});

   rx_buffer[0] = 0;
   rx_buffer[1] = 0x02;   // mismatch
   rx_buffer[2] = 0x02;
   rx_buffer[3] = 0b0001'1000;
   rx_buffer[4] = 0b0001'1000;
   rx_buffer[5] = 0x03;

   // trigger read burst config command
   sm.process_event(mpu6500::EventTick{});

   EXPECT_TRUE(sm.is(StateMachineDef::s_config_receive_wait));

   // indicate receive complete
   sm.process_event(mpu6500::EventReceiveDone{});
   EXPECT_TRUE(sm.is(StateMachineDef::s_verify_config));

   // provide tick for verification state to complete with fail
   sm.process_event(mpu6500::EventTick{});

   EXPECT_EQ(mpu6500_handler.get_error(), imu_sensor::ImuSensorError::none);
   EXPECT_FALSE(mpu6500_handler.config_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(ConfigSMTest, check_failed_config_due_to_config_mismatch)
{
   // trigger write burst config command
   sm.process_event(mpu6500::EventTick{});

   rx_buffer[0] = 0;
   rx_buffer[1] = 0x03;
   rx_buffer[2] = 0x03;   // mismatch
   rx_buffer[3] = 0b0001'1000;
   rx_buffer[4] = 0b0001'1000;
   rx_buffer[5] = 0x03;

   // trigger read burst config command
   sm.process_event(mpu6500::EventTick{});

   EXPECT_TRUE(sm.is(StateMachineDef::s_config_receive_wait));

   // indicate receive complete
   sm.process_event(mpu6500::EventReceiveDone{});
   EXPECT_TRUE(sm.is(StateMachineDef::s_verify_config));

   // provide tick for verification state to complete with fail
   sm.process_event(mpu6500::EventTick{});

   EXPECT_EQ(mpu6500_handler.get_error(), imu_sensor::ImuSensorError::none);
   EXPECT_FALSE(mpu6500_handler.config_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(ConfigSMTest, check_failed_config_due_to_gyro_config_mismatch)
{
   // trigger write burst config command
   sm.process_event(mpu6500::EventTick{});

   rx_buffer[0] = 0;
   rx_buffer[1] = 0x03;
   rx_buffer[2] = 0x02;
   rx_buffer[3] = 0b0001'0000;   // mismatch
   rx_buffer[4] = 0b0001'1000;
   rx_buffer[5] = 0x03;

   // trigger read burst config command
   sm.process_event(mpu6500::EventTick{});

   EXPECT_TRUE(sm.is(StateMachineDef::s_config_receive_wait));

   // indicate receive complete
   sm.process_event(mpu6500::EventReceiveDone{});
   EXPECT_TRUE(sm.is(StateMachineDef::s_verify_config));

   // provide tick for verification state to complete with fail
   sm.process_event(mpu6500::EventTick{});

   EXPECT_EQ(mpu6500_handler.get_error(), imu_sensor::ImuSensorError::none);
   EXPECT_FALSE(mpu6500_handler.config_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(ConfigSMTest, check_failed_config_due_to_accel_config_mismatch)
{
   // trigger write burst config command
   sm.process_event(mpu6500::EventTick{});

   rx_buffer[0] = 0;
   rx_buffer[1] = 0x03;
   rx_buffer[2] = 0x02;
   rx_buffer[3] = 0b0001'1000;
   rx_buffer[4] = 0b0000'1000;   // mismatch
   rx_buffer[5] = 0x03;

   // trigger read burst config command
   sm.process_event(mpu6500::EventTick{});

   EXPECT_TRUE(sm.is(StateMachineDef::s_config_receive_wait));

   // indicate receive complete
   sm.process_event(mpu6500::EventReceiveDone{});
   EXPECT_TRUE(sm.is(StateMachineDef::s_verify_config));

   // provide tick for verification state to complete with fail
   sm.process_event(mpu6500::EventTick{});

   EXPECT_EQ(mpu6500_handler.get_error(), imu_sensor::ImuSensorError::none);
   EXPECT_FALSE(mpu6500_handler.config_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(ConfigSMTest, check_failed_config_due_to_accel_config_2_mismatch)
{
   // trigger write burst config command
   sm.process_event(mpu6500::EventTick{});

   rx_buffer[0] = 0;
   rx_buffer[1] = 0x03;
   rx_buffer[2] = 0x02;
   rx_buffer[3] = 0b0001'1000;
   rx_buffer[4] = 0b0001'1000;
   rx_buffer[5] = 0x30;   // mismatch

   // trigger read burst config command
   sm.process_event(mpu6500::EventTick{});

   EXPECT_TRUE(sm.is(StateMachineDef::s_config_receive_wait));

   // indicate receive complete
   sm.process_event(mpu6500::EventReceiveDone{});
   EXPECT_TRUE(sm.is(StateMachineDef::s_verify_config));

   // provide tick for verification state to complete with fail
   sm.process_event(mpu6500::EventTick{});

   EXPECT_EQ(mpu6500_handler.get_error(), imu_sensor::ImuSensorError::none);
   EXPECT_FALSE(mpu6500_handler.config_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(ConfigSMTest, check_successful_config_read_back)
{
   // trigger write burst config command
   sm.process_event(mpu6500::EventTick{});

   rx_buffer[0] = 0;
   rx_buffer[1] = 0x03;
   rx_buffer[2] = 0xe2;          // switch random bits -> should be masked out
   rx_buffer[3] = 0b1101'1010;   // switch random bits -> should be masked out
   rx_buffer[4] = 0b1111'1101;   // switch random bits -> should be masked out
   rx_buffer[5] = 0xf3;          // switch random bits -> should be masked out

   // trigger read burst config command
   sm.process_event(mpu6500::EventTick{});

   EXPECT_TRUE(sm.is(StateMachineDef::s_config_receive_wait));

   // indicate receive complete
   sm.process_event(mpu6500::EventReceiveDone{});
   EXPECT_TRUE(sm.is(StateMachineDef::s_verify_config));

   // provide tick for verification state to complete with fail
   sm.process_event(mpu6500::EventTick{});

   EXPECT_EQ(mpu6500_handler.get_error(), imu_sensor::ImuSensorError::none);
   EXPECT_TRUE(mpu6500_handler.config_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

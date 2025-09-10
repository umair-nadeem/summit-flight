#include "mpu6500/ResetSM.hpp"

#include "Mpu6500BaseTest.hpp"
#include "mpu6500/Mpu6500StateHandler.hpp"

class ResetSMTest : public Mpu6500BaseTest
{
protected:
   using StateHandler    = mpu6500::Mpu6500StateHandler<sys_time::ClockSource, decltype(spi_master_with_dma), mocks::common::Logger>;
   using StateMachineDef = mpu6500::ResetStateMachine<StateHandler>;

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
                                accel_range_plausibility_margin_mps2};

   boost::sml::sm<StateMachineDef> sm{mpu6500_handler};
};

TEST_F(ResetSMTest, check_power_reset_state)
{
   EXPECT_TRUE(sm.is(StateMachineDef::s_power_reset));
}

TEST_F(ResetSMTest, check_power_reset_command)
{
   sm.process_event(mpu6500::EventTick{});

   test_buffer[0] = mpu6500::params::pwr_mgmt_1_reg;
   test_buffer[1] = 0x80;

   EXPECT_THAT(tx_buffer, testing::ElementsAreArray(test_buffer.begin(), test_buffer.end()));
   EXPECT_TRUE(sm.is(StateMachineDef::s_power_reset_wait));

   const std::size_t wait_ticks_needed = mpu6500::params::power_on_reset_wait_ms / execution_period_ms;
   for (std::size_t i = 0; i < wait_ticks_needed; i++)
   {
      sm.process_event(mpu6500::EventTick{});
   }

   // still in power reset wait state
   EXPECT_TRUE(sm.is(StateMachineDef::s_power_reset_wait));
}

TEST_F(ResetSMTest, check_signal_path_reset_command)
{
   sm.process_event(mpu6500::EventTick{});   // triggers power reset
   const std::size_t power_reset_wait_ticks_needed = mpu6500::params::power_on_reset_wait_ms / execution_period_ms;
   for (std::size_t i = 0; i < power_reset_wait_ticks_needed; i++)
   {
      sm.process_event(mpu6500::EventTick{});
   }

   // still in power reset wait state
   EXPECT_TRUE(sm.is(StateMachineDef::s_power_reset_wait));

   // move to signal reset state
   sm.process_event(mpu6500::EventTick{});
   EXPECT_TRUE(sm.is(StateMachineDef::s_signal_reset));

   // perform signal reset
   sm.process_event(mpu6500::EventTick{});
   test_buffer[0] = mpu6500::params::signal_path_reset_reg;
   test_buffer[1] = 0x7;

   EXPECT_THAT(tx_buffer, testing::ElementsAreArray(test_buffer.begin(), test_buffer.end()));
   EXPECT_TRUE(sm.is(StateMachineDef::s_signal_reset_wait));

   const std::size_t signal_reset_wait_ticks_needed = mpu6500::params::signal_path_reset_wait_ms / execution_period_ms;
   for (std::size_t i = 0; i < signal_reset_wait_ticks_needed; i++)
   {
      sm.process_event(mpu6500::EventTick{});
   }

   // still in signal reset wait state
   EXPECT_TRUE(sm.is(StateMachineDef::s_signal_reset_wait));

   // state machine should terminate
   sm.process_event(mpu6500::EventTick{});
   EXPECT_TRUE(sm.is(boost::sml::X));
}

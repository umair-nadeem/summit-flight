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
                                accel_range_plausibility_margin_mps2};

   boost::sml::sm<StateMachineDef> sm{mpu6500_handler};
};

TEST_F(SelfTestSMTest, check_s_dlpf_config_state)
{
   EXPECT_TRUE(sm.is(StateMachineDef::s_dlpf_config));
}

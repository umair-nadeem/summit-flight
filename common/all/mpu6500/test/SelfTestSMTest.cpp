#include "mpu6500/SelfTestSM.hpp"

#include "Mpu6500BaseTest.hpp"
#include "mpu6500/Mpu6500StateHandler.hpp"

class SelfTestSMTest : public Mpu6500BaseTest
{
protected:
   using StateHandler    = mpu6500::Mpu6500StateHandler<sys_time::ClockSource, decltype(spi_master_with_dma)>;
   using StateMachineDef = mpu6500::SelfTestStateMachine<StateHandler>;

   StateHandler                    mpu6500_handler{imu_data, spi_master_with_dma, execution_period_ms, receive_wait_timeout_ms};
   boost::sml::sm<StateMachineDef> sm{mpu6500_handler};
};

TEST_F(SelfTestSMTest, check_s_dlpf_config_state)
{
   EXPECT_TRUE(sm.is(StateMachineDef::s_dlpf_config));
}

#include "bmp390/SetupSM.hpp"

#include "Bmp390BaseTest.hpp"
#include "bmp390/Bmp390StateHandler.hpp"

class Bmp390SetupSMTest : public Bmp390BaseTest
{
protected:
   using StateHandler    = bmp390::Bmp390StateHandler<sys_time::ClockSource, decltype(i2c_driver), mocks::common::Logger>;
   using StateMachineDef = bmp390::SetupStateMachine<StateHandler>;

   StateHandler state_handler{barometer_data_storage,
                              barometer_health_storage,
                              i2c_driver,
                              logger,
                              read_failures_limit,
                              execution_period_ms,
                              receive_wait_timeout_ms};

   boost::sml::sm<StateMachineDef> sm{state_handler};
};

TEST_F(Bmp390SetupSMTest, check_initial_state)
{
   EXPECT_TRUE(sm.is(StateMachineDef::s_idle));
}

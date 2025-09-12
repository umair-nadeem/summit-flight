#include "bmp390/SetupSM.hpp"

#include "Bmp390BaseTest.hpp"
#include "bmp390/Bmp390StateHandler.hpp"

class Bmp390SetupSMTest : public Bmp390BaseTest
{
protected:
   void provide_ticks(const std::size_t num_ticks)
   {
      for (std::size_t i = 0; i < num_ticks; i++)
      {
         sm.process_event(bmp390::EventTick{});
      }
   }

   using StateHandler    = bmp390::Bmp390StateHandler<sys_time::ClockSource, decltype(i2c_driver), mocks::common::Logger>;
   using StateMachineDef = bmp390::SetupStateMachine<StateHandler>;

   StateHandler state_handler{barometer_data_storage,
                              barometer_health_storage,
                              i2c_driver,
                              logger,
                              read_failures_limit,
                              max_recovery_attempts,
                              execution_period_ms,
                              receive_wait_timeout_ms};

   boost::sml::sm<StateMachineDef> sm{state_handler};
};

TEST_F(Bmp390SetupSMTest, check_initial_state)
{
   EXPECT_TRUE(sm.is(StateMachineDef::s_idle));
}

TEST_F(Bmp390SetupSMTest, check_soft_reset_fail_with_bus_error)
{
   i2c_driver.m_transaction_result = false;
   sm.process_event(bmp390::EventTick{});

   std::array ref_tx_buffer{bmp390::params::cmd_reg, bmp390::params::CmdReg::soft_reset};
   EXPECT_THAT(ref_tx_buffer, testing::ElementsAreArray(i2c_driver.m_tx_buffer_span));

   sm.process_event(bmp390::EventTick{});   // bus error evaluated

   barometer_sensor::ErrorBits ref_error{};
   ref_error.set(static_cast<uint8_t>(barometer_sensor::BarometerSensorError::bus_error));
   EXPECT_EQ(state_handler.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(Bmp390SetupSMTest, check_read_id_fail_with_bus_error)
{
   // soft reset successful
   sm.process_event(bmp390::EventTick{});

   std::array ref_tx_buffer{bmp390::params::cmd_reg, bmp390::params::CmdReg::soft_reset};
   EXPECT_THAT(ref_tx_buffer, testing::ElementsAreArray(i2c_driver.m_tx_buffer_span));

   // read id command fails with bus error
   i2c_driver.m_transaction_result = false;
   sm.process_event(bmp390::EventTick{});   // read id command executed
   sm.process_event(bmp390::EventTick{});   // bus error evaluated

   barometer_sensor::ErrorBits ref_error{};
   ref_error.set(static_cast<uint8_t>(barometer_sensor::BarometerSensorError::bus_error));

   EXPECT_EQ(state_handler.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(i2c_driver.m_read_reg.value(), bmp390::params::chip_id_reg);
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(Bmp390SetupSMTest, check_read_id_fail_with_timeout)
{
   // soft reset successful
   sm.process_event(bmp390::EventTick{});

   std::array ref_tx_buffer{bmp390::params::cmd_reg, bmp390::params::CmdReg::soft_reset};
   EXPECT_THAT(ref_tx_buffer, testing::ElementsAreArray(i2c_driver.m_tx_buffer_span));

   // read id command fails with timeout
   sm.process_event(bmp390::EventTick{});                                 // read id command executed
   provide_ticks((receive_wait_timeout_ms / execution_period_ms) + 1u);   // timeout ticks

   barometer_sensor::ErrorBits ref_error{};
   ref_error.set(static_cast<uint8_t>(barometer_sensor::BarometerSensorError::bus_error));

   EXPECT_EQ(state_handler.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(i2c_driver.m_read_reg.value(), bmp390::params::chip_id_reg);
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(Bmp390SetupSMTest, check_read_id_fail_with_id_mismatch)
{
   // soft reset successful
   sm.process_event(bmp390::EventTick{});

   std::array ref_tx_buffer{bmp390::params::cmd_reg, bmp390::params::CmdReg::soft_reset};
   EXPECT_THAT(ref_tx_buffer, testing::ElementsAreArray(i2c_driver.m_tx_buffer_span));

   std::array<uint8_t, 1> ref_rx_buffer{0};
   i2c_driver.stage_rx_buffer(ref_rx_buffer);
   // read id command fails with id mismatch
   sm.process_event(bmp390::EventTick{});   // read id command executed
   sm.process_event(bmp390::EventReceiveDone{});

   barometer_sensor::ErrorBits ref_error{};
   ref_error.set(static_cast<uint8_t>(barometer_sensor::BarometerSensorError::id_mismatch_error));

   EXPECT_EQ(state_handler.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(i2c_driver.m_read_reg.value(), bmp390::params::chip_id_reg);
   EXPECT_FALSE(state_handler.setup_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(Bmp390SetupSMTest, check_config_write_fail_with_bus_error)
{
   // soft reset successful
   sm.process_event(bmp390::EventTick{});

   std::array<uint8_t, 1> ref_rx_buffer{bmp390::params::device_id};
   i2c_driver.stage_rx_buffer(ref_rx_buffer);
   // read id succeeds
   sm.process_event(bmp390::EventTick{});          // read id command executed
   sm.process_event(bmp390::EventReceiveDone{});   // id read successfully

   EXPECT_EQ(state_handler.get_error().to_ulong(), 0);

   // config writes will fail with bus error
   i2c_driver.m_transaction_result = false;

   // osr register
   sm.process_event(bmp390::EventTick{});

   std::array ref_tx_buffer = {bmp390::params::osr_reg, static_cast<uint8_t>((osr4_t << 3u) | osr_p)};
   EXPECT_THAT(ref_tx_buffer, testing::ElementsAreArray(i2c_driver.m_tx_buffer_span));

   // odr register
   sm.process_event(bmp390::EventTick{});

   ref_tx_buffer = {bmp390::params::odr_reg, odr_sel};
   EXPECT_THAT(ref_tx_buffer, testing::ElementsAreArray(i2c_driver.m_tx_buffer_span));

   // iir register
   sm.process_event(bmp390::EventTick{});

   ref_tx_buffer = {bmp390::params::config_reg, iir_filter << 1u};
   EXPECT_THAT(ref_tx_buffer, testing::ElementsAreArray(i2c_driver.m_tx_buffer_span));

   sm.process_event(bmp390::EventTick{});   // bus error evaluated

   barometer_sensor::ErrorBits ref_error{};
   ref_error.set(static_cast<uint8_t>(barometer_sensor::BarometerSensorError::bus_error));

   EXPECT_EQ(state_handler.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_FALSE(state_handler.setup_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(Bmp390SetupSMTest, check_power_mode_with_bus_error)
{
   // soft reset successful
   sm.process_event(bmp390::EventTick{});

   std::array<uint8_t, 1> ref_rx_buffer{bmp390::params::device_id};
   i2c_driver.stage_rx_buffer(ref_rx_buffer);
   // read id succeeds
   sm.process_event(bmp390::EventTick{});          // read id command executed
   sm.process_event(bmp390::EventReceiveDone{});   // id read successfully

   EXPECT_EQ(state_handler.get_error().to_ulong(), 0);

   sm.process_event(bmp390::EventTick{});   // osr
   sm.process_event(bmp390::EventTick{});   // odr
   sm.process_event(bmp390::EventTick{});   // iir

   i2c_driver.m_transaction_result = false;
   sm.process_event(bmp390::EventTick{});   // power control

   std::array ref_tx_buffer = {bmp390::params::pwr_ctrl_reg, get_pwr_ctrl()};
   EXPECT_THAT(ref_tx_buffer, testing::ElementsAreArray(i2c_driver.m_tx_buffer_span));

   sm.process_event(bmp390::EventTick{});   // bus error evaluated

   barometer_sensor::ErrorBits ref_error{};
   ref_error.set(static_cast<uint8_t>(barometer_sensor::BarometerSensorError::bus_error));

   EXPECT_EQ(state_handler.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_FALSE(state_handler.setup_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(Bmp390SetupSMTest, check_read_config_burst_fail_with_timeout)
{
   // soft reset successful
   sm.process_event(bmp390::EventTick{});

   std::array<uint8_t, 1> ref_rx_buffer{bmp390::params::device_id};
   i2c_driver.stage_rx_buffer(ref_rx_buffer);
   // read id succeeds
   sm.process_event(bmp390::EventTick{});          // read id command executed
   sm.process_event(bmp390::EventReceiveDone{});   // id read successfully

   EXPECT_EQ(state_handler.get_error().to_ulong(), 0);

   sm.process_event(bmp390::EventTick{});                                 // osr
   sm.process_event(bmp390::EventTick{});                                 // odr
   sm.process_event(bmp390::EventTick{});                                 // iir
   sm.process_event(bmp390::EventTick{});                                 // power control
   sm.process_event(bmp390::EventTick{});                                 // read config burst

   provide_ticks((receive_wait_timeout_ms / execution_period_ms) + 1u);   // timeout ticks

   barometer_sensor::ErrorBits ref_error{};
   ref_error.set(static_cast<uint8_t>(barometer_sensor::BarometerSensorError::bus_error));

   EXPECT_EQ(state_handler.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_FALSE(state_handler.setup_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(Bmp390SetupSMTest, check_read_config_burst_fail_with_pwr_ctrl_mismatch)
{
   // soft reset successful
   sm.process_event(bmp390::EventTick{});

   std::array<uint8_t, 1> ref_rx_buffer{bmp390::params::device_id};
   i2c_driver.stage_rx_buffer(ref_rx_buffer);
   // read id succeeds
   sm.process_event(bmp390::EventTick{});          // read id command executed
   sm.process_event(bmp390::EventReceiveDone{});   // id read successfully

   EXPECT_EQ(state_handler.get_error().to_ulong(), 0);

   sm.process_event(bmp390::EventTick{});   // osr
   sm.process_event(bmp390::EventTick{});   // odr
   sm.process_event(bmp390::EventTick{});   // iir
   sm.process_event(bmp390::EventTick{});   // power control

   // read config burst fails with power control mismatch
   std::array<uint8_t, 5> ref_config_rx_buffer{0,                                              // wrong pwr ctrl
                                               static_cast<uint8_t>((osr4_t << 3u) | osr_p),   // osr
                                               odr_sel,                                        // odr
                                               0,                                              // reserved
                                               iir_filter << 1u};                              // iir

   i2c_driver.stage_rx_buffer(ref_config_rx_buffer);
   // read config burst
   sm.process_event(bmp390::EventTick{});
   sm.process_event(bmp390::EventReceiveDone{});

   barometer_sensor::ErrorBits ref_error{};
   ref_error.set(static_cast<uint8_t>(barometer_sensor::BarometerSensorError::config_mismatch_error));

   EXPECT_EQ(state_handler.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_FALSE(state_handler.setup_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(Bmp390SetupSMTest, check_read_config_burst_fail_with_odr_mismatch)
{
   // soft reset successful
   sm.process_event(bmp390::EventTick{});

   std::array<uint8_t, 1> ref_rx_buffer{bmp390::params::device_id};
   i2c_driver.stage_rx_buffer(ref_rx_buffer);
   // read id succeeds
   sm.process_event(bmp390::EventTick{});          // read id command executed
   sm.process_event(bmp390::EventReceiveDone{});   // id read successfully

   EXPECT_EQ(state_handler.get_error().to_ulong(), 0);

   sm.process_event(bmp390::EventTick{});   // osr
   sm.process_event(bmp390::EventTick{});   // odr
   sm.process_event(bmp390::EventTick{});   // iir
   sm.process_event(bmp390::EventTick{});   // power control

   // read config burst fails with power control mismatch
   std::array<uint8_t, 5> ref_config_rx_buffer{get_pwr_ctrl(),                                 // pwr ctrl
                                               static_cast<uint8_t>((osr4_t << 3u) | osr_p),   // osr
                                               0,                                              // wrong odr
                                               0,                                              // reserved
                                               iir_filter << 1u};                              // iir

   i2c_driver.stage_rx_buffer(ref_config_rx_buffer);
   // read config burst
   sm.process_event(bmp390::EventTick{});
   sm.process_event(bmp390::EventReceiveDone{});

   barometer_sensor::ErrorBits ref_error{};
   ref_error.set(static_cast<uint8_t>(barometer_sensor::BarometerSensorError::config_mismatch_error));

   EXPECT_EQ(state_handler.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_FALSE(state_handler.setup_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

TEST_F(Bmp390SetupSMTest, check_read_config_burst_successful)
{
   // soft reset successful
   sm.process_event(bmp390::EventTick{});

   std::array<uint8_t, 1> ref_rx_buffer{bmp390::params::device_id};
   i2c_driver.stage_rx_buffer(ref_rx_buffer);
   // read id succeeds
   sm.process_event(bmp390::EventTick{});          // read id command executed
   sm.process_event(bmp390::EventReceiveDone{});   // id read successfully

   EXPECT_EQ(state_handler.get_error().to_ulong(), 0);

   sm.process_event(bmp390::EventTick{});   // osr
   sm.process_event(bmp390::EventTick{});   // odr
   sm.process_event(bmp390::EventTick{});   // iir
   sm.process_event(bmp390::EventTick{});   // power control

   // read config burst succeeds
   std::array<uint8_t, 5> ref_config_rx_buffer{get_pwr_ctrl(),                                 // pwr ctrl
                                               static_cast<uint8_t>((osr4_t << 3u) | osr_p),   // osr
                                               odr_sel,                                        // odr
                                               0,                                              // reserved
                                               iir_filter << 1u};                              // iir

   i2c_driver.stage_rx_buffer(ref_config_rx_buffer);
   // read config burst
   sm.process_event(bmp390::EventTick{});
   sm.process_event(bmp390::EventReceiveDone{});

   EXPECT_EQ(state_handler.get_error().to_ulong(), 0);
   EXPECT_TRUE(state_handler.setup_successful());
   EXPECT_TRUE(sm.is(boost::sml::X));
}

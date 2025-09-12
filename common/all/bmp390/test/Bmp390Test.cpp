#include "bmp390/Bmp390.hpp"

#include "Bmp390BaseTest.hpp"

class Bmp390Test : public Bmp390BaseTest
{
protected:
   void provide_ticks(const std::size_t num_ticks)
   {
      for (std::size_t i = 0; i < num_ticks; i++)
      {
         bmp390.execute();
      }
   }

   void run_through_setup_state(const bool success)
   {
      i2c_driver.m_transaction_result = success;

      if (success)
      {
         bmp390.execute();   // soft reset
         std::array<uint8_t, 1> ref_rx_buffer{bmp390::params::device_id};
         i2c_driver.stage_rx_buffer(ref_rx_buffer);

         bmp390.execute();   // read id
         bmp390.notify_receive_complete();

         bmp390.execute();   // osr
         bmp390.execute();   // odr
         bmp390.execute();   // iir
         bmp390.execute();   // pwr ctrl

         // read config burst succeeds
         std::array<uint8_t, 5> ref_config_rx_buffer{get_pwr_ctrl(),                                 // pwr ctrl
                                                     static_cast<uint8_t>((osr4_t << 3u) | osr_p),   // osr
                                                     odr_sel,                                        // odr
                                                     0,                                              // reserved
                                                     iir_filter << 1u};                              // iir

         i2c_driver.stage_rx_buffer(ref_config_rx_buffer);
         // read config burst
         bmp390.execute();
         bmp390.notify_receive_complete();
      }
      else
      {
         bmp390.execute();   // soft reset
         bmp390.execute();   // evaluate bus error
      }
   }

   void provide_valid_coefficients()
   {
      // perform valid coefficients readout
      i2c_driver.stage_rx_buffer(trimming_coefficients_buffer);
      bmp390.execute();   // read coefficients
      bmp390.notify_receive_complete();
   }

   void provide_valid_measurement()
   {
      std::array<uint8_t, bmp390::params::num_bytes_data> ref_data_buffer{0, 0, 2u, 2u, 99u, 10u, 99u, 119u};
      i2c_driver.stage_rx_buffer(ref_data_buffer);

      bmp390.execute();   // triggers read command
      bmp390.notify_receive_complete();

      // sensor data
      const auto barometer_data = barometer_data_storage.get_latest().data;
      EXPECT_NEAR(barometer_data.temperature_c.value(), 12.65f, 0.1f);
      EXPECT_NEAR(barometer_data.pressure_pa, 100367.758f, 0.1f);
      EXPECT_NEAR(barometer_data.altitude_m.value(), 55.85, 0.1f);
   }

   void fail_operation_state_due_to_read_timeout()
   {
      std::array<uint8_t, bmp390::params::num_bytes_data> empty_data_buffer{};
      i2c_driver.stage_rx_buffer(empty_data_buffer);

      for (size_t i = 0; i < read_failures_limit; i++)
      {
         bmp390.execute();   // triggers read command
         provide_ticks((receive_wait_timeout_ms / execution_period_ms) + 1u);
      }
   }

   bmp390::Bmp390<sys_time::ClockSource, decltype(i2c_driver), mocks::common::Logger> bmp390{barometer_data_storage,
                                                                                             barometer_health_storage,
                                                                                             i2c_driver,
                                                                                             logger,
                                                                                             read_failures_limit,
                                                                                             max_recovery_attempts,
                                                                                             execution_period_ms,
                                                                                             receive_wait_timeout_ms};
};

TEST_F(Bmp390Test, check_setup_failure)
{
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::stopped);

   // move to reset state
   bmp390.start();

   // run through reset sub-state machine
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::setup);
   run_through_setup_state(false);

   barometer_sensor::BarometerHealth::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(barometer_sensor::BarometerSensorError::bus_error));

   EXPECT_EQ(bmp390.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::failure);
}

TEST_F(Bmp390Test, check_setup_success)
{
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::stopped);

   // move to reset state
   bmp390.start();

   // run through reset sub-state machine
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::setup);
   run_through_setup_state(true);

   EXPECT_EQ(bmp390.get_error().to_ulong(), 0);
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::read_coefficients);

   const auto barometer_health = barometer_health_storage.get_latest().data;
   EXPECT_EQ(barometer_health.state, barometer_sensor::BarometerSensorState::read_coefficients);
   EXPECT_TRUE(barometer_health.error.none());
   EXPECT_EQ(barometer_health.read_failure_count, 0);
   EXPECT_TRUE(barometer_health.setup_ok);
}

TEST_F(Bmp390Test, check_read_coefficients_fail_with_timeout)
{
   bmp390.start();
   run_through_setup_state(true);

   bmp390.execute();   // read coefficients
   provide_ticks((receive_wait_timeout_ms / execution_period_ms) + 1u);

   barometer_sensor::BarometerHealth::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(barometer_sensor::BarometerSensorError::bus_error));

   EXPECT_EQ(bmp390.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(i2c_driver.m_read_reg, bmp390::params::calibration_data_reg);
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::failure);
}

TEST_F(Bmp390Test, check_read_coefficients_all_zeros)
{
   bmp390.start();
   run_through_setup_state(true);

   // all zero calibration coefficients
   std::array<uint8_t, bmp390::params::num_bytes_calibration_data> ref_rx_buffer{0};
   i2c_driver.stage_rx_buffer(ref_rx_buffer);

   bmp390.execute();   // read coefficients
   bmp390.notify_receive_complete();

   barometer_sensor::BarometerHealth::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(barometer_sensor::BarometerSensorError::coefficients_pattern_error));

   EXPECT_EQ(bmp390.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(i2c_driver.m_read_reg, bmp390::params::calibration_data_reg);
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::failure);
}

TEST_F(Bmp390Test, check_read_coefficients_all_ones)
{
   bmp390.start();
   run_through_setup_state(true);

   // all zero calibration coefficients
   std::array<uint8_t, bmp390::params::num_bytes_calibration_data> ref_rx_buffer{};
   ref_rx_buffer.fill(0xff);
   i2c_driver.stage_rx_buffer(ref_rx_buffer);

   bmp390.execute();   // read coefficients
   bmp390.notify_receive_complete();

   barometer_sensor::BarometerHealth::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(barometer_sensor::BarometerSensorError::coefficients_pattern_error));

   EXPECT_EQ(bmp390.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(i2c_driver.m_read_reg, bmp390::params::calibration_data_reg);
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::failure);
}

TEST_F(Bmp390Test, check_read_data_timeout_causing_recovery)
{
   bmp390.start();
   run_through_setup_state(true);

   bmp390.execute();   // read coefficients
   bmp390.notify_receive_complete();

   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::operational);

   fail_operation_state_due_to_read_timeout();

   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::recovery);
   EXPECT_EQ(i2c_driver.m_read_reg, bmp390::params::err_reg);   // read from 0x02 register

   barometer_sensor::BarometerHealth::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(barometer_sensor::BarometerSensorError::bus_error));

   const auto barometer_health = barometer_health_storage.get_latest().data;
   EXPECT_EQ(barometer_health.state, barometer_sensor::BarometerSensorState::recovery);
   EXPECT_EQ(barometer_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(barometer_health.read_failure_count, read_failures_limit);
   EXPECT_TRUE(barometer_health.setup_ok);
}

TEST_F(Bmp390Test, check_all_zero_data_causing_recovery)
{
   bmp390.start();
   run_through_setup_state(true);

   bmp390.execute();   // read coefficients
   bmp390.notify_receive_complete();

   // all zero data
   std::array<uint8_t, bmp390::params::num_bytes_data> ref_rx_buffer{0};
   i2c_driver.stage_rx_buffer(ref_rx_buffer);

   for (size_t i = 0; i < read_failures_limit; i++)
   {
      bmp390.execute();   // triggers read command
      bmp390.notify_receive_complete();
   }

   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::recovery);
   EXPECT_EQ(i2c_driver.m_read_reg, bmp390::params::err_reg);   // read from 0x02 register

   barometer_sensor::BarometerHealth::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(barometer_sensor::BarometerSensorError::data_pattern_error));

   const auto barometer_health = barometer_health_storage.get_latest().data;
   EXPECT_EQ(barometer_health.state, barometer_sensor::BarometerSensorState::recovery);
   EXPECT_EQ(barometer_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(barometer_health.read_failure_count, read_failures_limit);
   EXPECT_TRUE(barometer_health.setup_ok);
}

TEST_F(Bmp390Test, check_all_ones_data_causing_recovery)
{
   bmp390.start();
   run_through_setup_state(true);

   bmp390.execute();   // read coefficients
   bmp390.notify_receive_complete();

   // all ones data
   std::array<uint8_t, bmp390::params::num_bytes_data> ref_rx_buffer{0};
   ref_rx_buffer.fill(0xff);
   ref_rx_buffer[0] = 0;   // err reg must be zero otherwise sensor error will result
   i2c_driver.stage_rx_buffer(ref_rx_buffer);

   for (size_t i = 0; i < read_failures_limit; i++)
   {
      bmp390.execute();   // triggers read command
      bmp390.notify_receive_complete();
   }

   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::recovery);
   EXPECT_EQ(i2c_driver.m_read_reg, bmp390::params::err_reg);   // read from 0x02 register

   barometer_sensor::BarometerHealth::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(barometer_sensor::BarometerSensorError::data_pattern_error));

   const auto barometer_health = barometer_health_storage.get_latest().data;
   EXPECT_EQ(barometer_health.state, barometer_sensor::BarometerSensorState::recovery);
   EXPECT_EQ(barometer_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(barometer_health.read_failure_count, read_failures_limit);
   EXPECT_TRUE(barometer_health.setup_ok);
}

TEST_F(Bmp390Test, check_err_reg_bit_causing_recovery)
{
   bmp390.start();
   run_through_setup_state(true);

   bmp390.execute();   // read coefficients
   bmp390.notify_receive_complete();

   std::array error_bits = {bmp390::params::ErrReg::fatal_err_mask,
                            bmp390::params::ErrReg::cmd_err_mask,
                            bmp390::params::ErrReg::conf_err_mask};

   std::array<uint8_t, bmp390::params::num_bytes_data> ref_rx_buffer{0};
   std::size_t                                         bit = 0;
   for (size_t i = 0; i < read_failures_limit; i++)
   {
      ref_rx_buffer[0] = error_bits[((bit++) % error_bits.size())];   // err_reg
      ref_rx_buffer[1] = 0;                                           // status reg -> don't care
      ref_rx_buffer[2] = 1u;                                          // data byte -> must be non-zero
      i2c_driver.stage_rx_buffer(ref_rx_buffer);

      bmp390.execute();                                               // triggers read command
      bmp390.notify_receive_complete();
   }

   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::recovery);
   EXPECT_EQ(i2c_driver.m_read_reg, bmp390::params::err_reg);   // read from 0x02 register

   barometer_sensor::BarometerHealth::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(barometer_sensor::BarometerSensorError::sensor_error));

   const auto barometer_health = barometer_health_storage.get_latest().data;
   EXPECT_EQ(barometer_health.state, barometer_sensor::BarometerSensorState::recovery);
   EXPECT_EQ(barometer_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(barometer_health.read_failure_count, read_failures_limit);
   EXPECT_TRUE(barometer_health.setup_ok);
}

TEST_F(Bmp390Test, check_below_range_data_causing_recovery)
{
   bmp390.start();
   run_through_setup_state(true);

   bmp390.execute();   // read coefficients
   bmp390.notify_receive_complete();

   std::array<uint8_t, bmp390::params::num_bytes_data> ref_rx_buffer{0, 0, 1u};
   i2c_driver.stage_rx_buffer(ref_rx_buffer);

   for (size_t i = 0; i < read_failures_limit; i++)
   {
      bmp390.execute();   // triggers read command
      bmp390.notify_receive_complete();
   }

   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::recovery);
   EXPECT_EQ(i2c_driver.m_read_reg, bmp390::params::err_reg);   // read from 0x02 register

   barometer_sensor::BarometerHealth::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(barometer_sensor::BarometerSensorError::out_of_range_data_error));

   // sensor health
   const auto barometer_health = barometer_health_storage.get_latest().data;
   EXPECT_EQ(barometer_health.state, barometer_sensor::BarometerSensorState::recovery);
   EXPECT_EQ(barometer_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(barometer_health.read_failure_count, read_failures_limit);
   EXPECT_TRUE(barometer_health.setup_ok);
}

TEST_F(Bmp390Test, check_above_range_data_causing_recovery)
{
   bmp390.start();
   run_through_setup_state(true);

   std::array<uint8_t, bmp390::params::num_bytes_calibration_data> ref_coef_buffer{};
   ref_coef_buffer.fill(0xfe);
   ref_coef_buffer[0] = 0;   // par_t1 low -> temperature high
   ref_coef_buffer[1] = 0;   // par_t1 low -> temperature high
   i2c_driver.stage_rx_buffer(ref_coef_buffer);

   bmp390.execute();         // read coefficients
   bmp390.notify_receive_complete();

   std::array<uint8_t, bmp390::params::num_bytes_data> ref_data_buffer{};
   ref_data_buffer.fill(0xfe);
   ref_data_buffer[0] = 0;   // err reg must be zero otherwise sensor error will result
   i2c_driver.stage_rx_buffer(ref_data_buffer);

   for (size_t i = 0; i < read_failures_limit; i++)
   {
      bmp390.execute();   // triggers read command
      bmp390.notify_receive_complete();
   }

   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::recovery);
   EXPECT_EQ(i2c_driver.m_read_reg, bmp390::params::err_reg);   // read from 0x02 register

   barometer_sensor::BarometerHealth::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(barometer_sensor::BarometerSensorError::out_of_range_data_error));

   const auto barometer_health = barometer_health_storage.get_latest().data;
   EXPECT_EQ(barometer_health.state, barometer_sensor::BarometerSensorState::recovery);
   EXPECT_EQ(barometer_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(barometer_health.read_failure_count, read_failures_limit);
   EXPECT_TRUE(barometer_health.setup_ok);
}

TEST_F(Bmp390Test, check_valid_data_readout)
{
   bmp390.start();
   run_through_setup_state(true);
   provide_valid_coefficients();
   provide_valid_measurement();

   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::operational);

   // sensor health
   const auto barometer_health = barometer_health_storage.get_latest().data;
   EXPECT_EQ(barometer_health.state, barometer_sensor::BarometerSensorState::operational);
   EXPECT_EQ(barometer_health.error.to_ulong(), 0);
   EXPECT_EQ(barometer_health.read_failure_count, 0);
   EXPECT_TRUE(barometer_health.setup_ok);
}

TEST_F(Bmp390Test, check_recovery_successful_in_last_attempt)
{
   bmp390.start();
   run_through_setup_state(true);
   provide_valid_coefficients();
   provide_valid_measurement();

   fail_operation_state_due_to_read_timeout();

   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::recovery);

   for (std::size_t i = 0; i < (max_recovery_attempts - 1u); i++)
   {
      bmp390.execute();   // enter recovery setup sm
      run_through_setup_state(false);
      const auto health = barometer_health_storage.get_latest().data;
      EXPECT_EQ(health.recovery_attempt_count, i + 1u);
      EXPECT_FALSE(health.setup_ok);
   }

   bmp390.execute();   // make last recovery attempt
   run_through_setup_state(true);

   // sensor health
   barometer_sensor::BarometerHealth::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(barometer_sensor::BarometerSensorError::bus_error));

   const auto barometer_health = barometer_health_storage.get_latest().data;
   EXPECT_EQ(barometer_health.state, barometer_sensor::BarometerSensorState::operational);
   EXPECT_EQ(barometer_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(barometer_health.read_failure_count, read_failures_limit);   // read failure count is not reset until valid measurement
   EXPECT_EQ(barometer_health.recovery_attempt_count, 0);
   EXPECT_TRUE(barometer_health.setup_ok);

   provide_valid_measurement();

   const auto final_barometer_health = barometer_health_storage.get_latest().data;
   EXPECT_EQ(barometer_health.state, barometer_sensor::BarometerSensorState::operational);
   EXPECT_EQ(final_barometer_health.read_failure_count, 0);   // read failure count is reset
}

TEST_F(Bmp390Test, check_recovery_to_failure)
{
   bmp390.start();
   run_through_setup_state(true);
   provide_valid_coefficients();
   provide_valid_measurement();

   fail_operation_state_due_to_read_timeout();

   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::recovery);

   for (std::size_t i = 0; i < max_recovery_attempts; i++)
   {
      bmp390.execute();   // enter recovery setup sm
      run_through_setup_state(false);
      const auto health = barometer_health_storage.get_latest().data;
      EXPECT_EQ(health.recovery_attempt_count, i + 1u);
   }

   // sensor health
   barometer_sensor::BarometerHealth::ErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(barometer_sensor::BarometerSensorError::bus_error));

   const auto barometer_health = barometer_health_storage.get_latest().data;
   EXPECT_EQ(barometer_health.state, barometer_sensor::BarometerSensorState::failure);
   EXPECT_EQ(barometer_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(barometer_health.read_failure_count, read_failures_limit);   // read failure count is not reset until valid measurement
   EXPECT_EQ(barometer_health.recovery_attempt_count, max_recovery_attempts);
   EXPECT_FALSE(barometer_health.setup_ok);
}

TEST_F(Bmp390Test, stop_immediately_after_entering_setup)
{
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::stopped);
   bmp390.start();
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::setup);
   bmp390.stop();
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::stopped);
}

TEST_F(Bmp390Test, stop_during_setup)
{
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::stopped);
   bmp390.start();
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::setup);
   bmp390.execute();
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::setup);
   bmp390.stop();
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::stopped);
}

TEST_F(Bmp390Test, stop_after_setup)
{
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::stopped);
   bmp390.start();
   run_through_setup_state(true);
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::read_coefficients);
   bmp390.stop();
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::stopped);
}

TEST_F(Bmp390Test, stop_during_operational)
{
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::stopped);
   bmp390.start();
   run_through_setup_state(true);
   provide_valid_coefficients();
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::operational);
   bmp390.stop();
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::stopped);
}

TEST_F(Bmp390Test, stop_during_recovery)
{
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::stopped);
   bmp390.start();
   run_through_setup_state(true);
   provide_valid_coefficients();
   fail_operation_state_due_to_read_timeout();
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::recovery);
   bmp390.stop();
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::stopped);
}

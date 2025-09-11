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
         i2c_driver.m_transaction_result = false;
         bmp390.execute();   // soft reset
         bmp390.execute();   // evaluate bus error
      }
   }

   bmp390::Bmp390<sys_time::ClockSource, decltype(i2c_driver), mocks::common::Logger> bmp390{barometer_data_storage,
                                                                                             barometer_health_storage,
                                                                                             i2c_driver,
                                                                                             logger,
                                                                                             read_failures_limit,
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
   ref_error.set(static_cast<uint8_t>(barometer_sensor::BarometerSensorError::bus_error));
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
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::setup);
}

TEST_F(Bmp390Test, check_read_coefficients_fail_with_timeout)
{
   bmp390.start();
   run_through_setup_state(true);

   bmp390.execute();   // read coefficients
   provide_ticks((receive_wait_timeout_ms / execution_period_ms) + 1u);

   barometer_sensor::BarometerHealth::ErrorBits ref_error{};
   ref_error.set(static_cast<uint8_t>(barometer_sensor::BarometerSensorError::bus_error));
   EXPECT_EQ(bmp390.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(i2c_driver.m_read_reg, bmp390::params::calibration_data_reg);
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::failure);
}

TEST_F(Bmp390Test, check_read_coefficients_zero_values)
{
   bmp390.start();
   run_through_setup_state(true);

   // all zero calibration coefficients
   std::array<uint8_t, bmp390::params::num_bytes_calibration_data> ref_rx_buffer{0};
   i2c_driver.stage_rx_buffer(ref_rx_buffer);

   bmp390.execute();   // read coefficients
   bmp390.notify_receive_complete();

   barometer_sensor::BarometerHealth::ErrorBits ref_error{};
   ref_error.set(static_cast<uint8_t>(barometer_sensor::BarometerSensorError::coefficients_error));
   EXPECT_EQ(bmp390.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(i2c_driver.m_read_reg, bmp390::params::calibration_data_reg);
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::failure);
}

TEST_F(Bmp390Test, check_read_data_timeout)
{
   bmp390.start();
   run_through_setup_state(true);

   bmp390.execute();   // read coefficients
   bmp390.notify_receive_complete();

   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::operational);

   for (size_t i = 0; i < read_failures_limit; i++)
   {
      bmp390.execute();   // triggers read command
      bmp390.execute();   // causes timer to increment
      bmp390.execute();   // causes timeout
   }

   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::soft_recovery);

   barometer_sensor::BarometerHealth::ErrorBits ref_error{};
   ref_error.set(static_cast<uint8_t>(barometer_sensor::BarometerSensorError::bus_error));
   EXPECT_EQ(bmp390.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(i2c_driver.m_read_reg, bmp390::params::err_reg);   // read from 0x02 register
   EXPECT_EQ(bmp390.get_state(), barometer_sensor::BarometerSensorState::failure);
}

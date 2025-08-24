#include "mpu6500/Mpu6500.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "Mpu6500BaseTest.hpp"
#include "mocks/hw/SpiMasterWithDma.hpp"
#include "sys_time/ClockSource.hpp"

class Mpu6500Test : public Mpu6500BaseTest
{
protected:
   void run_through_reset_state()
   {
      EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::reset);

      const std::size_t wait_ticks_needed = (mpu6500::params::power_on_reset_wait_ms + mpu6500::params::signal_path_reset_wait_ms) / execution_period_ms;
      for (std::size_t i = 0; i < wait_ticks_needed; i++)
      {
         mpu6500.execute();
      }

      mpu6500.execute();
      mpu6500.execute();
      mpu6500.execute();
      mpu6500.execute();
   }

   void run_through_validation_state(const bool validation_success)
   {
      EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::validation);

      mpu6500.execute();
      mpu6500.execute();

      if (validation_success)
      {
         rx_buffer[1] = mpu6500::params::device_id;
         mpu6500.execute();
         mpu6500.notify_receive_complete();
         mpu6500.execute();
      }
      else
      {
         mpu6500.execute();
         mpu6500.execute();
         mpu6500.execute();
      }
   }

   void run_through_self_test_state()
   {
      EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::self_test);

      mpu6500.execute();
      mpu6500.execute();
   }

   void run_through_config_state(const bool config_success)
   {
      EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::config);

      mpu6500.execute();

      if (config_success)
      {
         rx_buffer[0] = 0;
         rx_buffer[1] = 0x03;
         rx_buffer[2] = 0xe2;
         rx_buffer[3] = 0b1101'1010;
         rx_buffer[4] = 0b1111'1101;
         rx_buffer[5] = 0xf3;

         mpu6500.execute();
         mpu6500.notify_receive_complete();
         mpu6500.execute();
      }
      else
      {
         mpu6500.execute();
         mpu6500.execute();
         mpu6500.execute();
      }
   }

   mpu6500::Mpu6500<sys_time::ClockSource, decltype(spi_master_with_dma)> mpu6500{imu_data_storage,
                                                                                  imu_health_storage,
                                                                                  spi_master_with_dma,
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
};

TEST_F(Mpu6500Test, check_reset_process)
{
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::stopped);

   // move to reset state
   mpu6500.start();

   // run through reset sub-state machine
   run_through_reset_state();

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::validation);
}

TEST_F(Mpu6500Test, check_failed_validation)
{
   // run through reset sub-state machine
   mpu6500.start();
   run_through_reset_state();

   // run through validation sub-state machine with failure
   run_through_validation_state(false);

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::failure);
}

TEST_F(Mpu6500Test, check_successful_validation)
{
   // run through reset sub-state machine
   mpu6500.start();
   run_through_reset_state();

   // run through validation sub-state machine with failure
   run_through_validation_state(true);

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::self_test);
}

TEST_F(Mpu6500Test, check_self_test)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);

   run_through_self_test_state();

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::config);
}

TEST_F(Mpu6500Test, check_failed_config)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();

   run_through_config_state(false);

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::failure);
}

TEST_F(Mpu6500Test, check_successful_config)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();

   run_through_config_state(true);

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::operational);

   const auto imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::operational);
   EXPECT_EQ(imu_health.error, imu_sensor::ImuSensorError::none);
   EXPECT_EQ(imu_health.read_failure_count, 0);
   EXPECT_TRUE(imu_health.validation_ok);
   EXPECT_TRUE(imu_health.self_test_ok);
   EXPECT_TRUE(imu_health.config_ok);
}

TEST_F(Mpu6500Test, check_read_data_command)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();
   run_through_config_state(true);
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::operational);

   mpu6500.execute();

   test_buffer[0] = 0x80 | mpu6500::params::accel_xout_h_reg;
   EXPECT_THAT(tx_buffer, testing::ElementsAreArray(test_buffer.begin(), test_buffer.end()));
}

TEST_F(Mpu6500Test, check_read_data_timeout_causing_soft_rec)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();
   run_through_config_state(true);
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::operational);

   for (size_t i = 0; i < read_failures_limit; i++)
   {
      mpu6500.execute();   // triggers read command
      mpu6500.execute();   // causes timer to increment
      mpu6500.execute();   // causes timeout
      EXPECT_EQ(mpu6500.get_error(), imu_sensor::ImuSensorError::bus_error);
   }

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::soft_recovery);

   const auto imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::soft_recovery);
   EXPECT_EQ(imu_health.error, imu_sensor::ImuSensorError::bus_error);
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);
}

TEST_F(Mpu6500Test, check_all_zeros_data_causing_soft_rec)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();
   run_through_config_state(true);
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::operational);

   rx_buffer.fill(0);
   for (size_t i = 0; i < read_failures_limit; i++)
   {
      mpu6500.execute();                   // triggers read command
      mpu6500.notify_receive_complete();   // trigger data validation
      EXPECT_EQ(mpu6500.get_error(), imu_sensor::ImuSensorError::sensor_error);
   }

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::soft_recovery);

   const auto imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::soft_recovery);
   EXPECT_EQ(imu_health.error, imu_sensor::ImuSensorError::sensor_error);
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);
}

TEST_F(Mpu6500Test, check_combination_wait_timeout_and_all_zeros_data_causing_soft_rec)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();
   run_through_config_state(true);
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::operational);

   rx_buffer.fill(0);

   mpu6500.execute();                   // triggers read command
   mpu6500.execute();                   // causes timer to increment
   mpu6500.execute();                   // causes timeout
   EXPECT_EQ(mpu6500.get_error(), imu_sensor::ImuSensorError::bus_error);

   mpu6500.execute();                   // triggers read command
   mpu6500.notify_receive_complete();   // trigger data validation
   EXPECT_EQ(mpu6500.get_error(), imu_sensor::ImuSensorError::sensor_error);

   mpu6500.execute();                   // triggers read command
   mpu6500.execute();                   // causes timer to increment
   mpu6500.execute();                   // causes timeout
   EXPECT_EQ(mpu6500.get_error(), imu_sensor::ImuSensorError::bus_error);

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::soft_recovery);

   const auto imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::soft_recovery);
   EXPECT_EQ(imu_health.error, imu_sensor::ImuSensorError::bus_error);
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);
}

TEST_F(Mpu6500Test, check_temp_implausibility_causing_soft_rec)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();
   run_through_config_state(true);
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::operational);

   rx_buffer.fill(0);
   // read_failures_limit = 3

   rx_buffer[7] = 0xc2;                 // temperature->-26 degC
   rx_buffer[8] = 0xb4;
   mpu6500.execute();                   // triggers read command
   mpu6500.notify_receive_complete();   // trigger data validation
   EXPECT_EQ(mpu6500.get_error(), imu_sensor::ImuSensorError::data_error);

   rx_buffer[7] = 0x54;                 // temperature->+86 degC
   rx_buffer[8] = 0xc6;
   mpu6500.execute();                   // triggers read command
   mpu6500.notify_receive_complete();   // trigger data validation
   EXPECT_EQ(mpu6500.get_error(), imu_sensor::ImuSensorError::data_error);

   rx_buffer[7] = 0xc2;                 // temperature->+86 degC
   rx_buffer[8] = 0xb4;
   mpu6500.execute();                   // triggers read command
   mpu6500.notify_receive_complete();   // trigger data validation
   EXPECT_EQ(mpu6500.get_error(), imu_sensor::ImuSensorError::data_error);

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::soft_recovery);

   const auto imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::soft_recovery);
   EXPECT_EQ(imu_health.error, imu_sensor::ImuSensorError::data_error);
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);
}

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
      rx_buffer.fill(0);
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
      rx_buffer.fill(0);
      mpu6500.execute();

      if (config_success)
      {
         rx_buffer[0] = 0;                            // address
         rx_buffer[1] = sample_rate_divider;          // sample rate div
         rx_buffer[2] = dlpf_config;                  // config
         rx_buffer[3] = (gyro_full_scale << 3u);      // gyro config
         rx_buffer[4] = (accel_full_scale << 3u);     // accel config
         rx_buffer[5] = 0xf0 | accel_a_dlpf_config;   // accel config 2, f -> dummy bits

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

   void fail_operational_state_bus_sensor_data_error_to_soft_recovery()
   {
      EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::operational);

      rx_buffer.fill(0);

      mpu6500.execute();                   // triggers read command
      mpu6500.execute();                   // causes timer to increment
      mpu6500.execute();                   // causes timeout
      EXPECT_TRUE(mpu6500.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::bus_error)));

      mpu6500.execute();                   // triggers read command
      mpu6500.notify_receive_complete();   // trigger data validation
      EXPECT_TRUE(mpu6500.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::sensor_error)));

      rx_buffer[7] = 0x54;                 // temperature->+86 degC
      rx_buffer[8] = 0xc6;
      mpu6500.execute();                   // triggers read command
      mpu6500.notify_receive_complete();   // trigger data validation
      EXPECT_TRUE(mpu6500.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_error)));

      EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::soft_recovery);
   }

   void perform_valid_measurement()
   {
      EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::operational);

      rx_buffer.fill(0);

      rx_buffer[1]  = 0xe0;                // ACCEL_XOUT_H  (-8192) -1 g
      rx_buffer[2]  = 0x00;                // ACCEL_XOUT_L
      rx_buffer[3]  = 0x10;                // ACCEL_YOUT_H  (+4096) 0.5 g
      rx_buffer[4]  = 0x00;                // ACCEL_YOUT_L
      rx_buffer[5]  = 0x20;                // ACCEL_ZOUT_H  (+8192) +1 g
      rx_buffer[6]  = 0x00;                // ACCEL_ZOUT_L
      rx_buffer[7]  = 0x18;                // TEMP_OUT_H    (+6343) temperature->+40 degC
      rx_buffer[8]  = 0xc7;                // TEMP_OUT_L
      rx_buffer[9]  = 0xf9;                // GYRO_XOUT_H   (-1640)-50 dps
      rx_buffer[10] = 0x98;                // GYRO_XOUT_L
      rx_buffer[11] = 0x0c;                // GYRO_YOUT_H   (+3280) 100 dps
      rx_buffer[12] = 0xd0;                // GYRO_YOUT_L
      rx_buffer[13] = 0x19;                // GYRO_ZOUT_H   (+6560) 200 dps
      rx_buffer[14] = 0xa0;                // GYRO_ZOUT_L

      mpu6500.execute();                   // triggers read command
      mpu6500.notify_receive_complete();   // trigger data validation

      EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::operational);

      const auto imu_data = imu_data_storage.get_latest().data;

      EXPECT_NEAR(imu_data.accel_mps2.x, -9.80665f, 0.01f);
      EXPECT_NEAR(imu_data.accel_mps2.y, 4.9033f, 0.01f);
      EXPECT_NEAR(imu_data.accel_mps2.z, 9.80665f, 0.01f);
      EXPECT_NEAR(imu_data.gyro_radps.x, -0.873f, 0.01f);
      EXPECT_NEAR(imu_data.gyro_radps.y, 1.746f, 0.01f);
      EXPECT_NEAR(imu_data.gyro_radps.z, 3.492f, 0.01f);
      EXPECT_NEAR(imu_data.temperature_c.value(), 40.0f, 0.01f);
   }

   mpu6500::Mpu6500<sys_time::ClockSource, decltype(spi_master_with_dma), mocks::common::Logger> mpu6500{imu_data_storage,
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
};

TEST_F(Mpu6500Test, check_reset_process)
{
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::stopped);

   // move to reset state
   mpu6500.start();

   // run through reset sub-state machine
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::reset);
   run_through_reset_state();

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::validation);
}

TEST_F(Mpu6500Test, check_failed_validation)
{
   // run through reset sub-state machine
   mpu6500.start();
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::reset);
   run_through_reset_state();

   // run through validation sub-state machine with failure
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::validation);
   run_through_validation_state(false);

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::failure);
}

TEST_F(Mpu6500Test, check_successful_validation)
{
   // run through reset sub-state machine
   mpu6500.start();
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::reset);
   run_through_reset_state();

   // run through validation sub-state machine with failure
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::validation);
   run_through_validation_state(true);

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::self_test);
}

TEST_F(Mpu6500Test, check_self_test)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::reset);
   run_through_reset_state();
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::validation);
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

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::config);
   run_through_config_state(false);   // timeout/bus error

   EXPECT_TRUE(mpu6500.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::bus_error)));
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::failure);
}

TEST_F(Mpu6500Test, check_successful_config)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::config);
   run_through_config_state(true);

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::operational);

   const auto imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::operational);
   EXPECT_TRUE(imu_health.error.none());
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
      EXPECT_TRUE(mpu6500.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::bus_error)));
   }

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::soft_recovery);

   const auto imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::soft_recovery);
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::bus_error)));
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
      EXPECT_TRUE(mpu6500.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::sensor_error)));
   }

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::soft_recovery);

   const auto imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::soft_recovery);
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::sensor_error)));
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
   EXPECT_TRUE(mpu6500.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_error)));

   rx_buffer[7] = 0x54;                 // temperature->+86 degC
   rx_buffer[8] = 0xc6;
   mpu6500.execute();                   // triggers read command
   mpu6500.notify_receive_complete();   // trigger data validation
   EXPECT_TRUE(mpu6500.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_error)));

   rx_buffer[7] = 0xc2;                 // temperature->+86 degC
   rx_buffer[8] = 0xb4;
   mpu6500.execute();                   // triggers read command
   mpu6500.notify_receive_complete();   // trigger data validation
   EXPECT_TRUE(mpu6500.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_error)));

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::soft_recovery);

   const auto imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::soft_recovery);
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_error)));
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);
}

TEST_F(Mpu6500Test, check_valid_data_readout)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();
   run_through_config_state(true);

   perform_valid_measurement();
   EXPECT_TRUE(mpu6500.get_error().none());

   const auto imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::operational);
   EXPECT_TRUE(imu_health.error.none());
   EXPECT_EQ(imu_health.read_failure_count, 0);
}

TEST_F(Mpu6500Test, check_failing_validation_in_soft_rec_causing_hard_rec)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();
   run_through_config_state(true);

   fail_operational_state_bus_sensor_data_error_to_soft_recovery();

   auto imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::soft_recovery);
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_error)));
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);

   mpu6500.execute();                     // move into validation submachine
   run_through_validation_state(false);   // validation fails with timeout/bus error
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::hard_recovery);

   imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::hard_recovery);
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::bus_error)));
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_error)));
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::sensor_error)));
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);
}

TEST_F(Mpu6500Test, check_failing_config_in_soft_rec_causing_hard_rec)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();
   run_through_config_state(true);

   fail_operational_state_bus_sensor_data_error_to_soft_recovery();

   auto imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::soft_recovery);
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_error)));
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);

   mpu6500.execute();                    // move into validation submachine
   run_through_validation_state(true);   // validation successful
   run_through_config_state(false);      // config fails with timeout/bus error
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::hard_recovery);

   imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::hard_recovery);
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::bus_error)));
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_error)));
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::sensor_error)));
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);
}

TEST_F(Mpu6500Test, check_successful_soft_recovery)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();
   run_through_config_state(true);

   fail_operational_state_bus_sensor_data_error_to_soft_recovery();

   auto imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::soft_recovery);
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_error)));
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);

   mpu6500.execute();                    // move into validation submachine
   run_through_validation_state(true);   // validation successful
   run_through_config_state(true);       // config successful
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::operational);

   perform_valid_measurement();
   EXPECT_TRUE(mpu6500.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_error)));
   EXPECT_EQ(mpu6500.get_read_failure_count(), 0);   // read failure count is reset

   imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::operational);
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::bus_error)));
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_error)));
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::sensor_error)));
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);   // read failures are reset but not published
}

TEST_F(Mpu6500Test, check_failing_validation_in_hard_rec_causing_failure)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();
   run_through_config_state(true);

   fail_operational_state_bus_sensor_data_error_to_soft_recovery();

   auto imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::soft_recovery);
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_error)));
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);

   mpu6500.execute();                     // move into validation submachine
   run_through_validation_state(false);   // validation fails with timeout/bus error
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::hard_recovery);

   imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::hard_recovery);
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::bus_error)));
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);

   mpu6500.execute();   // move into reset submachine
   run_through_reset_state();
   run_through_validation_state(false);

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::failure);

   imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::failure);
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::bus_error)));
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_error)));
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::sensor_error)));
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);
}

TEST_F(Mpu6500Test, check_failing_config_in_hard_rec_causing_failure)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();
   run_through_config_state(true);

   fail_operational_state_bus_sensor_data_error_to_soft_recovery();

   auto imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::soft_recovery);
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_error)));
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);

   mpu6500.execute();                     // move into validation submachine
   run_through_validation_state(false);   // validation fails with timeout/bus error
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::hard_recovery);

   imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::hard_recovery);
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::bus_error)));
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);

   mpu6500.execute();                    // move into reset submachine
   run_through_reset_state();
   run_through_validation_state(true);   // validation successful
   run_through_config_state(false);      // config fails with timeout/bus error

   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::failure);

   imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::failure);
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::bus_error)));
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_error)));
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::sensor_error)));
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);
}

TEST_F(Mpu6500Test, check_successful_hard_recovery)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();
   run_through_config_state(true);

   fail_operational_state_bus_sensor_data_error_to_soft_recovery();

   auto imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::soft_recovery);
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_error)));
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);

   mpu6500.execute();                     // move into validation submachine
   run_through_validation_state(false);   // validation fails with timeout/bus error
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::hard_recovery);

   imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::hard_recovery);
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::bus_error)));
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);

   mpu6500.execute();                    // move into reset submachine
   run_through_reset_state();
   run_through_validation_state(true);   // validation successful
   run_through_config_state(true);       // config successful

   perform_valid_measurement();
   EXPECT_TRUE(mpu6500.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::bus_error)));
   EXPECT_EQ(mpu6500.get_read_failure_count(), 0);   // read failure count is reset

   imu_health = imu_health_storage.get_latest().data;
   EXPECT_EQ(imu_health.state, imu_sensor::ImuSensorState::operational);
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::bus_error)));
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_error)));
   EXPECT_TRUE(imu_health.error.test(static_cast<uint8_t>(imu_sensor::ImuSensorError::sensor_error)));
   EXPECT_EQ(imu_health.read_failure_count, read_failures_limit);   // read failures are reset but not published
}

TEST_F(Mpu6500Test, stop_in_reset)
{
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::stopped);
   mpu6500.start();
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::reset);
   mpu6500.stop();
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::stopped);
}

TEST_F(Mpu6500Test, stop_in_validation)
{
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::stopped);
   mpu6500.start();
   run_through_reset_state();
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::validation);
   mpu6500.stop();
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::stopped);
}

TEST_F(Mpu6500Test, stop_in_self_test)
{
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::stopped);
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::self_test);
   mpu6500.stop();
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::stopped);
}

TEST_F(Mpu6500Test, stop_in_config)
{
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::stopped);
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::config);
   mpu6500.stop();
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::stopped);
}

TEST_F(Mpu6500Test, stop_in_operational)
{
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::stopped);
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();
   run_through_config_state(true);
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::operational);
   mpu6500.stop();
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::stopped);
}

TEST_F(Mpu6500Test, stop_in_soft_recovery)
{
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::stopped);
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();
   run_through_config_state(true);
   fail_operational_state_bus_sensor_data_error_to_soft_recovery();
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::soft_recovery);
   mpu6500.stop();
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::stopped);
}

TEST_F(Mpu6500Test, stop_in_hard_recovery)
{
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::stopped);
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();
   run_through_config_state(true);
   fail_operational_state_bus_sensor_data_error_to_soft_recovery();
   mpu6500.execute();   // enter validation within soft recovery phase
   run_through_validation_state(false);
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::hard_recovery);
   mpu6500.stop();
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::stopped);
}

TEST_F(Mpu6500Test, stop_in_failure)
{
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::stopped);
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_self_test_state();
   run_through_config_state(true);
   fail_operational_state_bus_sensor_data_error_to_soft_recovery();
   mpu6500.execute();   // enter validation within soft recovery phase
   run_through_validation_state(false);
   mpu6500.execute();   // enter validation within hard recovery phase
   run_through_reset_state();
   run_through_validation_state(false);
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::failure);
   mpu6500.stop();   // do nothing
   EXPECT_EQ(mpu6500.get_state(), imu_sensor::ImuSensorState::failure);
}

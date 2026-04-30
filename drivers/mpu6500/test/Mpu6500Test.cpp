#include "mpu6500/Mpu6500.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "Mpu6500BaseTest.hpp"
#include "mocks/hw/SpiMasterWithDma.hpp"
#include "mocks/sys_time/ClockSource.hpp"

class Mpu6500Test : public Mpu6500BaseTest
{
protected:
   void run_through_reset_state()
   {
      const std::size_t wait_ticks_needed = (mpu6500::params::power_on_reset_wait_ms + mpu6500::params::signal_path_reset_wait_ms) / params.execution_period_ms;
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

   void run_through_config_state(const bool config_success)
   {
      rx_buffer.fill(0);
      mpu6500.execute();

      if (config_success)
      {
         rx_buffer[0] = 0;                                   // address
         rx_buffer[1] = params.sample_rate_divider;          // sample rate div
         rx_buffer[2] = params.dlpf_config;                  // config
         rx_buffer[3] = (params.gyro_full_scale << 3u);      // gyro config
         rx_buffer[4] = (params.accel_full_scale << 3u);     // accel config
         rx_buffer[5] = 0xf0 | params.accel_a_dlpf_config;   // accel config 2, f -> dummy bits

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

   void cause_bus_and_data_errors_to_soft_recovery()
   {
      EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::operational);

      rx_buffer.fill(0);

      mpu6500.execute();   // triggers read command
      mpu6500.execute();   // causes timer to increment
      mpu6500.execute();   // causes timeout

      imu_sensor::ImuSensorErrorBits ref_error{};
      ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::bus_error));
      EXPECT_EQ(mpu6500.get_error().to_ulong(), ref_error.to_ulong());

      mpu6500.execute();                   // triggers read command
      mpu6500.notify_receive_complete();   // trigger data validation
      ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::data_pattern_error));
      EXPECT_EQ(mpu6500.get_error().to_ulong(), ref_error.to_ulong());

      rx_buffer[7] = 0x54;                 // temperature->+86 degC
      rx_buffer[8] = 0xc6;
      mpu6500.execute();                   // triggers read command
      mpu6500.notify_receive_complete();   // trigger data validation
      ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::out_of_range_data_error));
      EXPECT_EQ(mpu6500.get_error().to_ulong(), ref_error.to_ulong());

      EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::soft_recovery);
   }

   void perform_valid_measurement()
   {
      EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::operational);

      rx_buffer.fill(0);

      provide_dynamic_accel_values();
      provide_dynamic_gyro_values();
      provide_valid_temp_values();

      mpu6500.execute();                   // triggers read command
      mpu6500.notify_receive_complete();   // trigger data validation

      EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::operational);

      const auto imu_data = imu_sensor_data;

      // compensate for bias of {−1.01,0.0,+0.70} m/s2 added during self test
      const float bias_x = -1.01f;
      const float bias_y = 0.0f;
      const float bias_z = 0.70f;
      EXPECT_NEAR(imu_data.accel_mps2[0], -9.80665f - bias_x, 0.01f);
      EXPECT_NEAR(imu_data.accel_mps2[1], 4.9033f - bias_y, 0.01f);
      EXPECT_NEAR(imu_data.accel_mps2[2], 9.80665f - bias_z, 0.01f);
      EXPECT_NEAR(imu_data.gyro_radps[0], -0.873f, 0.01f);
      EXPECT_NEAR(imu_data.gyro_radps[1], 1.746f, 0.01f);
      EXPECT_NEAR(imu_data.gyro_radps[2], 3.492f, 0.01f);
      EXPECT_NEAR(imu_data.temperature_c.value(), 40.0f, 0.01f);
   }

   mpu6500::Mpu6500<decltype(spi_master_with_dma), logging::Logger> mpu6500{spi_master_with_dma,
                                                                            logger,
                                                                            imu_sensor_data,
                                                                            imu_sensor_status,
                                                                            params};
};

TEST_F(Mpu6500Test, check_reset_process)
{
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::stopped);

   // move to reset state
   mpu6500.start();

   // run through reset sub-state machine
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::reset);
   run_through_reset_state();

   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::validation);
}

TEST_F(Mpu6500Test, check_failed_validation)
{
   // run through reset sub-state machine
   mpu6500.start();
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::reset);
   run_through_reset_state();

   // run through validation sub-state machine with failure
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::validation);
   run_through_validation_state(false);

   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::fault);
}

TEST_F(Mpu6500Test, check_successful_validation)
{
   // run through reset sub-state machine
   mpu6500.start();
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::reset);
   run_through_reset_state();

   // run through validation sub-state machine with failure
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::validation);
   run_through_validation_state(true);

   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::config);
}

TEST_F(Mpu6500Test, check_failed_config)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);

   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::config);
   run_through_config_state(false);   // timeout/bus error

   imu_sensor::ImuSensorErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::bus_error));
   EXPECT_EQ(mpu6500.get_error().to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::fault);
}

TEST_F(Mpu6500Test, check_successful_config)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);

   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::config);
   run_through_config_state(true);

   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::operational);

   const auto imu_health = imu_sensor_status;
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::operational);
   EXPECT_TRUE(imu_health.error.none());
   EXPECT_EQ(imu_health.read_failure_count, 0);
   EXPECT_TRUE(imu_health.validation_ok);
   EXPECT_TRUE(imu_health.config_ok);
}

TEST_F(Mpu6500Test, check_read_data_command)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_config_state(true);
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::operational);

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
   run_through_config_state(true);
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::operational);

   for (size_t i = 0; i < params.read_failures_limit; i++)
   {
      mpu6500.execute();   // triggers read command
      mpu6500.execute();   // causes timer to increment
      mpu6500.execute();   // causes timeout
      EXPECT_TRUE(mpu6500.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::bus_error)));
   }

   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::soft_recovery);

   const auto                     imu_health = imu_sensor_status;
   imu_sensor::ImuSensorErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::bus_error));
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::soft_recovery);
   EXPECT_EQ(imu_health.read_failure_count, params.read_failures_limit);
}

TEST_F(Mpu6500Test, check_all_zeros_data_causing_soft_rec)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_config_state(true);
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::operational);

   rx_buffer.fill(0);
   for (size_t i = 0; i < params.read_failures_limit; i++)
   {
      mpu6500.execute();                   // triggers read command
      mpu6500.notify_receive_complete();   // trigger data validation
      EXPECT_TRUE(mpu6500.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_pattern_error)));
   }

   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::soft_recovery);

   const auto                     imu_health = imu_sensor_status;
   imu_sensor::ImuSensorErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::data_pattern_error));
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::soft_recovery);
   EXPECT_EQ(imu_health.read_failure_count, params.read_failures_limit);
}

TEST_F(Mpu6500Test, check_all_ones_data_causing_soft_rec)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_config_state(true);
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::operational);

   rx_buffer.fill(0xff);
   for (size_t i = 0; i < params.read_failures_limit; i++)
   {
      mpu6500.execute();                   // triggers read command
      mpu6500.notify_receive_complete();   // trigger data validation
      EXPECT_TRUE(mpu6500.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::data_pattern_error)));
   }

   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::soft_recovery);

   const auto                     imu_health = imu_sensor_status;
   imu_sensor::ImuSensorErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::data_pattern_error));
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::soft_recovery);
   EXPECT_EQ(imu_health.read_failure_count, params.read_failures_limit);
}

TEST_F(Mpu6500Test, check_temp_implausibility_causing_soft_rec)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_config_state(true);
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::operational);

   rx_buffer.fill(0);
   // params.read_failures_limit = 3

   rx_buffer[7] = 0xc2;                 // temperature->-26 degC
   rx_buffer[8] = 0xb4;
   mpu6500.execute();                   // triggers read command
   mpu6500.notify_receive_complete();   // trigger data validation
   EXPECT_TRUE(mpu6500.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::out_of_range_data_error)));

   rx_buffer[7] = 0x54;                 // temperature->+86 degC
   rx_buffer[8] = 0xc6;
   mpu6500.execute();                   // triggers read command
   mpu6500.notify_receive_complete();   // trigger data validation
   EXPECT_TRUE(mpu6500.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::out_of_range_data_error)));

   rx_buffer[7] = 0xc2;                 // temperature->+86 degC
   rx_buffer[8] = 0xb4;
   mpu6500.execute();                   // triggers read command
   mpu6500.notify_receive_complete();   // trigger data validation
   EXPECT_TRUE(mpu6500.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::out_of_range_data_error)));

   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::soft_recovery);

   const auto                     imu_health = imu_sensor_status;
   imu_sensor::ImuSensorErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::out_of_range_data_error));
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::soft_recovery);
   EXPECT_EQ(imu_health.read_failure_count, params.read_failures_limit);
}

TEST_F(Mpu6500Test, check_valid_data_readout)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_config_state(true);

   perform_valid_measurement();
   EXPECT_TRUE(mpu6500.get_error().none());

   const auto imu_health = imu_sensor_status;
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::operational);
   EXPECT_TRUE(imu_health.error.none());
   EXPECT_EQ(imu_health.read_failure_count, 0);
}

TEST_F(Mpu6500Test, check_failing_validation_in_soft_rec_causing_hard_rec)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_config_state(true);

   cause_bus_and_data_errors_to_soft_recovery();

   auto                           imu_health = imu_sensor_status;
   imu_sensor::ImuSensorErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::bus_error));
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::data_pattern_error));
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::out_of_range_data_error));
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::soft_recovery);
   EXPECT_EQ(imu_health.read_failure_count, params.read_failures_limit);

   mpu6500.execute();                     // move into validation submachine
   run_through_validation_state(false);   // validation fails with timeout/bus error
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::hard_recovery);

   imu_health = imu_sensor_status;
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::hard_recovery);
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(imu_health.read_failure_count, params.read_failures_limit);
}

TEST_F(Mpu6500Test, check_failing_config_in_soft_rec_causing_hard_rec)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_config_state(true);

   cause_bus_and_data_errors_to_soft_recovery();

   auto imu_health = imu_sensor_status;
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::soft_recovery);
   imu_sensor::ImuSensorErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::bus_error));
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::data_pattern_error));
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::out_of_range_data_error));
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(imu_health.read_failure_count, params.read_failures_limit);

   mpu6500.execute();                    // move into validation submachine
   run_through_validation_state(true);   // validation successful
   run_through_config_state(false);      // config fails with timeout/bus error
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::hard_recovery);

   imu_health = imu_sensor_status;
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::hard_recovery);
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(imu_health.read_failure_count, params.read_failures_limit);
}

TEST_F(Mpu6500Test, check_successful_soft_recovery)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_config_state(true);

   cause_bus_and_data_errors_to_soft_recovery();

   auto                           imu_health = imu_sensor_status;
   imu_sensor::ImuSensorErrorBits ref_error{};
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::soft_recovery);
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::bus_error));
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::data_pattern_error));
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::out_of_range_data_error));
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(imu_health.read_failure_count, params.read_failures_limit);

   mpu6500.execute();                    // move into validation submachine
   run_through_validation_state(true);   // validation successful
   run_through_config_state(true);       // config successful
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::operational);

   perform_valid_measurement();
   EXPECT_TRUE(mpu6500.get_error().test(static_cast<uint8_t>(imu_sensor::ImuSensorError::out_of_range_data_error)));

   imu_health = imu_sensor_status;
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::operational);
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(imu_health.read_failure_count, 0);   // read failures are reset
}

TEST_F(Mpu6500Test, check_failing_validation_in_hard_rec_causing_failure)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_config_state(true);

   cause_bus_and_data_errors_to_soft_recovery();

   auto imu_health = imu_sensor_status;
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::soft_recovery);
   imu_sensor::ImuSensorErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::bus_error));
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::data_pattern_error));
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::out_of_range_data_error));
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(imu_health.read_failure_count, params.read_failures_limit);

   mpu6500.execute();                     // move into validation submachine
   run_through_validation_state(false);   // validation fails with timeout/bus error
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::hard_recovery);

   imu_health = imu_sensor_status;
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::hard_recovery);
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(imu_health.read_failure_count, params.read_failures_limit);

   mpu6500.execute();   // move into reset submachine
   run_through_reset_state();
   run_through_validation_state(false);

   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::fault);

   imu_health = imu_sensor_status;
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::fault);
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(imu_health.read_failure_count, params.read_failures_limit);
}

TEST_F(Mpu6500Test, check_failing_config_in_hard_rec_causing_failure)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_config_state(true);

   cause_bus_and_data_errors_to_soft_recovery();

   auto                           imu_health = imu_sensor_status;
   imu_sensor::ImuSensorErrorBits ref_error{};
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::bus_error));
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::data_pattern_error));
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::out_of_range_data_error));
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::soft_recovery);
   EXPECT_EQ(imu_health.read_failure_count, params.read_failures_limit);

   mpu6500.execute();                     // move into validation submachine
   run_through_validation_state(false);   // validation fails with timeout/bus error
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::hard_recovery);

   imu_health = imu_sensor_status;
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::hard_recovery);
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(imu_health.read_failure_count, params.read_failures_limit);

   mpu6500.execute();                    // move into reset submachine
   run_through_reset_state();
   run_through_validation_state(true);   // validation successful
   run_through_config_state(false);      // config fails with timeout/bus error

   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::fault);

   imu_health = imu_sensor_status;
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::fault);
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(imu_health.read_failure_count, params.read_failures_limit);
}

TEST_F(Mpu6500Test, check_successful_hard_recovery)
{
   // run through reset and validation sub-state machines
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_config_state(true);

   cause_bus_and_data_errors_to_soft_recovery();

   auto                           imu_health = imu_sensor_status;
   imu_sensor::ImuSensorErrorBits ref_error{};
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::soft_recovery);
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::bus_error));
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::data_pattern_error));
   ref_error.set(static_cast<uint32_t>(imu_sensor::ImuSensorError::out_of_range_data_error));
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(imu_health.read_failure_count, params.read_failures_limit);

   mpu6500.execute();                     // move into validation submachine
   run_through_validation_state(false);   // validation fails with timeout/bus error
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::hard_recovery);

   imu_health = imu_sensor_status;
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::hard_recovery);
   EXPECT_EQ(imu_health.read_failure_count, params.read_failures_limit);

   mpu6500.execute();                    // move into reset submachine
   run_through_reset_state();
   run_through_validation_state(true);   // validation successful
   run_through_config_state(true);       // config successful

   perform_valid_measurement();

   imu_health = imu_sensor_status;
   EXPECT_EQ(imu_health.error.to_ulong(), ref_error.to_ulong());
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::operational);
   EXPECT_EQ(imu_health.read_failure_count, 0);   // read failures are reset
}

TEST_F(Mpu6500Test, stop_in_reset)
{
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::stopped);
   mpu6500.start();
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::reset);
   mpu6500.stop();
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::stopped);
}

TEST_F(Mpu6500Test, stop_in_validation)
{
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::stopped);
   mpu6500.start();
   run_through_reset_state();
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::validation);
   mpu6500.stop();
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::stopped);
}

TEST_F(Mpu6500Test, stop_in_config)
{
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::stopped);
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::config);
   mpu6500.stop();
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::stopped);
}

TEST_F(Mpu6500Test, stop_in_operational)
{
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::stopped);
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_config_state(true);
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::operational);
   mpu6500.stop();
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::stopped);
}

TEST_F(Mpu6500Test, stop_in_soft_recovery)
{
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::stopped);
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_config_state(true);
   cause_bus_and_data_errors_to_soft_recovery();
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::soft_recovery);
   mpu6500.stop();
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::stopped);
}

TEST_F(Mpu6500Test, stop_in_hard_recovery)
{
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::stopped);
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_config_state(true);
   cause_bus_and_data_errors_to_soft_recovery();
   mpu6500.execute();   // enter validation within soft recovery phase
   run_through_validation_state(false);
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::hard_recovery);
   mpu6500.stop();
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::stopped);
}

TEST_F(Mpu6500Test, stop_in_failure)
{
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::stopped);
   mpu6500.start();
   run_through_reset_state();
   run_through_validation_state(true);
   run_through_config_state(true);
   cause_bus_and_data_errors_to_soft_recovery();
   mpu6500.execute();   // enter validation within soft recovery phase
   run_through_validation_state(false);
   mpu6500.execute();   // enter validation within hard recovery phase
   run_through_reset_state();
   run_through_validation_state(false);
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::fault);
   mpu6500.stop();   // do nothing
   EXPECT_EQ(mpu6500.get_state(), mpu6500::SensorState::fault);
}

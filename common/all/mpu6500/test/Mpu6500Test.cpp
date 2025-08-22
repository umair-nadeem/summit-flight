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
      EXPECT_EQ(mpu6500.get_state(), mpu6500::Mpu6500State::reset);

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
      EXPECT_EQ(mpu6500.get_state(), mpu6500::Mpu6500State::validation);

      mpu6500.execute();
      mpu6500.execute();

      if (validation_success)
      {
         rx_buffer[1] = mpu6500::params::device_id;
         mpu6500.execute();
         mpu6500.spi_transfer_complete_callback();
         mpu6500.execute();
      }
      else
      {
         mpu6500.execute();
         mpu6500.execute();
         mpu6500.execute();
      }
   }

   mpu6500::Mpu6500<sys_time::ClockSource, decltype(spi_master_with_dma)> mpu6500{imu_data,
                                                                                  spi_master_with_dma,
                                                                                  execution_period_ms,
                                                                                  receive_wait_timeout_ms};
};

TEST_F(Mpu6500Test, check_reset_process)
{
   EXPECT_EQ(mpu6500.get_state(), mpu6500::Mpu6500State::stopped);

   // move to reset state
   mpu6500.start();

   // run through reset sub-state machine
   run_through_reset_state();

   EXPECT_EQ(mpu6500.get_state(), mpu6500::Mpu6500State::validation);
}

TEST_F(Mpu6500Test, check_failed_validation)
{
   // run through reset sub-state machine
   mpu6500.start();
   run_through_reset_state();

   // run through validation sub-state machine with failure
   run_through_validation_state(false);

   EXPECT_EQ(mpu6500.get_state(), mpu6500::Mpu6500State::failure);
}

TEST_F(Mpu6500Test, check_successful_validation)
{
   // run through reset sub-state machine
   mpu6500.start();
   run_through_reset_state();

   // run through validation sub-state machine with failure
   run_through_validation_state(true);

   EXPECT_EQ(mpu6500.get_state(), mpu6500::Mpu6500State::self_test);
}

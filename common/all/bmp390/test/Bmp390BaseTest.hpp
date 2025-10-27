#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "barometer_sensor/BarometerData.hpp"
#include "barometer_sensor/BarometerHealth.hpp"
#include "bmp390/params.hpp"
#include "boundaries/SharedData.hpp"
#include "mocks/common/Logger.hpp"
#include "mocks/hw/I2c.hpp"
#include "sys_time/ClockSource.hpp"

class Bmp390BaseTest : public testing::Test
{
protected:
   static constexpr uint8_t get_pwr_ctrl()
   {
      return static_cast<uint8_t>(sensor_normal_mode << 4U) | static_cast<uint8_t>(1u << 1u) | 1u;
   }

   static constexpr uint8_t  read_failures_limit     = 3u;
   static constexpr uint8_t  max_recovery_attempts   = 3u;
   static constexpr uint32_t execution_period_ms     = 40u;
   static constexpr uint32_t receive_wait_timeout_ms = 80u;

   // parameter values
   static constexpr uint8_t osr_p              = 3u;
   static constexpr uint8_t osr4_t             = 0;
   static constexpr uint8_t odr_sel            = 3u;
   static constexpr uint8_t iir_filter         = 2u;
   static constexpr uint8_t sensor_normal_mode = 3u;

   boundaries::SharedData<barometer_sensor::BarometerData>   barometer_data_storage{};
   boundaries::SharedData<barometer_sensor::BarometerHealth> barometer_health_storage{};
   mocks::hw::I2c                                            i2c_driver{};
   mocks::common::Logger                                     logger{"bmp390_test"};

   std::array<uint8_t, bmp390::params::num_bytes_calibration_data>
       trimming_coefficients_buffer{214u, 108u, 213u, 76u, 249u, 234u, 26u, 252u, 20u, 6u,
                                    1u, 253u, 75u, 240u, 91u, 3u, 250u, 105u, 15u, 6u, 245};

   bmp390::params::CalibrationCoeffiecients raw_calibration_data{
       // temperature
       .par_t1  = 27862,
       .par_t2  = 19669,
       .par_t3  = -7,
       // pressure
       .par_p1  = 6890,
       .par_p2  = 5372,
       .par_p3  = 6,
       .par_p4  = 1,
       .par_p5  = 19453,
       .par_p6  = 23536,
       .par_p7  = 3,
       .par_p8  = -6,
       .par_p9  = 3945,
       .par_p10 = 6,
       .par_p11 = -11};
};

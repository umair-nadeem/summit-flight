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

   static constexpr uint8_t     read_failures_limit     = 3u;
   static constexpr std::size_t execution_period_ms     = 40u;
   static constexpr std::size_t receive_wait_timeout_ms = 80u;

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
};

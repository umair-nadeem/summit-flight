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
   static constexpr uint8_t     read_failures_limit     = 3u;
   static constexpr std::size_t execution_period_ms     = 40u;
   static constexpr std::size_t receive_wait_timeout_ms = 80u;

   // parameter values

   boundaries::SharedData<barometer_sensor::BarometerData>   barometer_data_storage{};
   boundaries::SharedData<barometer_sensor::BarometerHealth> barometer_health_storage{};
   mocks::hw::I2c                                            i2c_driver{};
   std::array<uint8_t, bmp390::params::buffer_size>          tx_buffer{};
   std::array<uint8_t, bmp390::params::buffer_size>          rx_buffer{};
   std::array<uint8_t, bmp390::params::buffer_size>          test_buffer{};
   mocks::common::Logger                                     logger{"bmp390_test"};
};

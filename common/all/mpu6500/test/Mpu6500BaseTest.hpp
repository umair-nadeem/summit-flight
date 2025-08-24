#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "boundaries/SharedData.hpp"
#include "imu_sensor/ImuData.hpp"
#include "imu_sensor/ImuHealth.hpp"
#include "mocks/hw/SpiMasterWithDma.hpp"
#include "mpu6500/params.hpp"
#include "sys_time/ClockSource.hpp"

class Mpu6500BaseTest : public testing::Test
{
protected:
   static constexpr std::size_t execution_period_ms     = 4u;
   static constexpr std::size_t receive_wait_timeout_ms = 4u;

   boundaries::SharedData<imu_sensor::ImuData>                 imu_data{};
   boundaries::SharedData<imu_sensor::ImuHealth>               imu_health{};
   std::array<uint8_t, mpu6500::params::num_bytes_transaction> tx_buffer{};
   std::array<uint8_t, mpu6500::params::num_bytes_transaction> rx_buffer{};
   std::array<uint8_t, mpu6500::params::num_bytes_transaction> test_buffer{};
   mocks::hw::SpiMasterWithDma                                 spi_master_with_dma{tx_buffer, rx_buffer};
};

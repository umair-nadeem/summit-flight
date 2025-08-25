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
   static constexpr uint8_t     read_failures_limit     = 3u;
   static constexpr std::size_t execution_period_ms     = 4u;
   static constexpr std::size_t receive_wait_timeout_ms = 4u;

   // parameter values
   static constexpr uint8_t sample_rate_divider                  = 0x03;
   static constexpr uint8_t dlpf_config                          = 0x02;
   static constexpr uint8_t gyro_full_scale                      = 0x02;
   static constexpr uint8_t accel_full_scale                     = 0x01;
   static constexpr uint8_t accel_a_dlpf_config                  = 0x03;
   static constexpr float   gyro_range_plausibility_margin_radps = 6.0f;
   static constexpr float   accel_range_plausibility_margin_mps2 = 20.0f;

   boundaries::SharedData<imu_sensor::ImuData>                 imu_data_storage{};
   boundaries::SharedData<imu_sensor::ImuHealth>               imu_health_storage{};
   std::array<uint8_t, mpu6500::params::num_bytes_transaction> tx_buffer{};
   std::array<uint8_t, mpu6500::params::num_bytes_transaction> rx_buffer{};
   std::array<uint8_t, mpu6500::params::num_bytes_transaction> test_buffer{};
   mocks::hw::SpiMasterWithDma                                 spi_master_with_dma{tx_buffer, rx_buffer};
};

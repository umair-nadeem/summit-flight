#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "boundaries/SharedData.hpp"
#include "imu_sensor/ImuData.hpp"
#include "imu_sensor/ImuHealth.hpp"
#include "mocks/common/Logger.hpp"
#include "mocks/hw/SpiMasterWithDma.hpp"
#include "mpu6500/params.hpp"
#include "sys_time/ClockSource.hpp"

class Mpu6500BaseTest : public testing::Test
{
protected:
   void provide_stationary_accel_values(const uint8_t random_offset = 0)
   {
      rx_buffer[1] = 0x17 + random_offset;   // accel -> 4.9 m/s2 + offset
      rx_buffer[2] = 0x00;
      rx_buffer[3] = 0x00 + random_offset;   // accel -> 0 m/s2 + offset
      rx_buffer[4] = 0x00;
      rx_buffer[5] = 0xf0;                   // accel -> -4.9 m/s2
      rx_buffer[6] = 0x00;
   }

   void provide_dynamic_accel_values()
   {
      rx_buffer[1] = 0xe0;   // ACCEL_XOUT_H  (-8192) -9.8 m/s2
      rx_buffer[2] = 0x00;   // ACCEL_XOUT_L
      rx_buffer[3] = 0x10;   // ACCEL_YOUT_H  (+4096) 4.9 m/s2
      rx_buffer[4] = 0x00;   // ACCEL_YOUT_L
      rx_buffer[5] = 0x20;   // ACCEL_ZOUT_H  (+8192) 9.8 m/s2
      rx_buffer[6] = 0x00;   // ACCEL_ZOUT_L
   }

   void provide_stationary_gyro_values(const uint8_t random_offset = 0)
   {
      rx_buffer[9]  = 0x00 + random_offset;   // gyro -> 0 rad/s + offset
      rx_buffer[10] = 0x00;
      rx_buffer[11] = 0x00 + random_offset;   // gyro -> 0 rad/s + offset
      rx_buffer[12] = 0x00;
      rx_buffer[13] = 0x00 + random_offset;   // gyro -> 0 rad/s + offset
      rx_buffer[14] = 0x00;
   }

   void provide_dynamic_gyro_values()
   {
      rx_buffer[9]  = 0xf9;   // GYRO_XOUT_H   (-1640)-50 dps
      rx_buffer[10] = 0x98;   // GYRO_XOUT_L
      rx_buffer[11] = 0x0c;   // GYRO_YOUT_H   (+3280) 100 dps
      rx_buffer[12] = 0xd0;   // GYRO_YOUT_L
      rx_buffer[13] = 0x19;   // GYRO_ZOUT_H   (+6560) 200 dps
      rx_buffer[14] = 0xa0;   // GYRO_ZOUT_L
   }

   void provide_valid_temp_values()
   {
      rx_buffer[7] = 0x18;   // TEMP_OUT_H    (+6343) temperature->+40 degC
      rx_buffer[8] = 0xc7;   // TEMP_OUT_L
   }

   static constexpr uint8_t  read_failures_limit     = 3u;
   static constexpr uint32_t execution_period_ms     = 4u;
   static constexpr uint32_t receive_wait_timeout_ms = 4u;

   // parameter values
   static constexpr uint8_t sample_rate_divider                  = 0x03;
   static constexpr uint8_t dlpf_config                          = 0x02;
   static constexpr uint8_t gyro_full_scale                      = 0x02;
   static constexpr uint8_t accel_full_scale                     = 0x01;
   static constexpr uint8_t accel_a_dlpf_config                  = 0x03;
   static constexpr float   gyro_range_plausibility_margin_radps = 6.0f;
   static constexpr float   accel_range_plausibility_margin_mps2 = 20.0f;
   static constexpr uint8_t num_samples_self_test                = 5u;
   static constexpr float   gyro_tolerance_radps                 = 0.1f;
   static constexpr float   accel_tolerance_mps2                 = 1.5f;

   boundaries::SharedData<imu_sensor::ImuData>                 imu_data_storage{};
   boundaries::SharedData<imu_sensor::ImuHealth>               imu_health_storage{};
   std::array<uint8_t, mpu6500::params::num_bytes_transaction> tx_buffer{};
   std::array<uint8_t, mpu6500::params::num_bytes_transaction> rx_buffer{};
   std::array<uint8_t, mpu6500::params::num_bytes_transaction> test_buffer{};
   mocks::hw::SpiMasterWithDma                                 spi_master_with_dma{tx_buffer, rx_buffer};
   mocks::common::Logger                                       logger{"mpu_test"};
};

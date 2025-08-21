#include "mpu6500/Mpu6500.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "mocks/hw/SpiMasterWithDma.hpp"
#include "sys_time/ClockSource.hpp"

class Mpu6500Test : public testing::Test
{
protected:
   void SetUp() override
   {
      mpu6500.initialize();
   }

   boundaries::SensorData<imu_sensor::ImuData>                            imu_data{};
   mocks::hw::SpiMasterWithDma<mpu6500::params::num_bytes_transaction>    spi_master_with_dma{};
   mpu6500::Mpu6500<sys_time::ClockSource, decltype(spi_master_with_dma)> mpu6500{imu_data, spi_master_with_dma};
};

TEST_F(Mpu6500Test, unnamed_test)
{
   mpu6500.execute();
}

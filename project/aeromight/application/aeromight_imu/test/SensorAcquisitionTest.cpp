#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "aeromight_imu/ImuDriverExecutor.hpp"
#include "mocks/common/Executable.hpp"
#include "mocks/common/Logger.hpp"
#include "mocks/pcb_component/Led.hpp"

class ImuDriverExecutorTest : public testing::Test
{
protected:
   mocks::common::Executable mpu_driver_mock{};
   mocks::pcb_component::Led led_mock{};
   mocks::common::Logger     logger{"test"};
   std::size_t               period{1u};

   aeromight_imu::ImuDriverExecutor<decltype(mpu_driver_mock),
                                    decltype(led_mock),
                                    decltype(logger)>
       imu_driver_executor{mpu_driver_mock,
                           led_mock,
                           logger,
                           period};
};

TEST_F(ImuDriverExecutorTest, test_run)
{
   imu_driver_executor.run_once();
}

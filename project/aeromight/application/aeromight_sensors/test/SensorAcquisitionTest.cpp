#include "aeromight_sensors/SensorAcquisition.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "mocks/common/Executable.hpp"
#include "mocks/common/Logger.hpp"
#include "mocks/pcb_component/Led.hpp"

class SensorAcquisitionTest : public testing::Test
{
protected:
   mocks::common::Executable mpu_driver_mock{};
   mocks::pcb_component::Led led_mock{};
   mocks::common::Logger     logger{};
   std::size_t               period{1u};

   aeromight_sensors::SensorAcquisition<decltype(mpu_driver_mock),
                                        decltype(led_mock),
                                        decltype(logger)>
       sensor_acquisition{mpu_driver_mock,
                          led_mock,
                          logger,
                          period};
};

TEST_F(SensorAcquisitionTest, canEventMsgWhenDeliveryContainerChanges)
{
   sensor_acquisition.run_once();
}

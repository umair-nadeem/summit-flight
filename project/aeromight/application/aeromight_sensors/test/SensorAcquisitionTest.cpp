#include "aeromight_sensors/SensorAcquisition.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

class SensorAcquisitionTest : public testing::Test
{
public:
   explicit SensorAcquisitionTest()
   {
   }

private:
   size_t                                       period{1u};
   aeromight_sensors::SensorAcquisition<size_t> sensor_acquisition{period, period};
};

TEST_F(SensorAcquisitionTest, canEventMsgWhenDeliveryContainerChanges)
{
   sensor_acquisition.run_once();
}

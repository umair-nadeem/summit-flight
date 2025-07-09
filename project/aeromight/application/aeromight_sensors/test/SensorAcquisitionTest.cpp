#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "aeromight_sensors/SensorAcquisition.hpp"

class SensorAcquisitionTest : public testing::Test
{
public:
   explicit SensorAcquisitionTest()
   {
   }

private:
   size_t value{125u};
};


TEST_F(SensorAcquisitionTest, canEventMsgWhenDeliveryContainerChanges)
{
}

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
   size_t value{125u};
};

TEST_F(SensorAcquisitionTest, canEventMsgWhenDeliveryContainerChanges)
{
}

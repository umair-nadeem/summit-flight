#include "aeromight_link/RadioLink.hpp"

#include <gmock/gmock.h>

#include "mocks/common/Executable.hpp"

class RadioLinkTest : public ::testing::Test
{
protected:
   static constexpr uint32_t execution_period_ms = 1u;

   mocks::common::Executable radio_receiver_mock{};
   mocks::common::Executable radio_transmitter_mock{};

   aeromight_link::RadioLink<decltype(radio_receiver_mock), decltype(radio_transmitter_mock)> radio_link{radio_receiver_mock,
                                                                                                         radio_transmitter_mock,
                                                                                                         execution_period_ms};
};

TEST_F(RadioLinkTest, check_execution)
{
   EXPECT_CALL(radio_receiver_mock, execute());
   EXPECT_CALL(radio_transmitter_mock, execute());

   radio_link.run_once();
}

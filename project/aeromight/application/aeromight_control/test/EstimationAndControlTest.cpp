#include "aeromight_control/EstimationAndControl.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "mocks/common/Executable.hpp"

class EstimationAndControlTest : public testing::Test
{
protected:
   static constexpr uint32_t period_in_ms = 8u;

   mocks::common::Executable                                                                  estimation_mock{};
   mocks::common::Executable                                                                  control_mock{};
   aeromight_control::EstimationAndControl<decltype(estimation_mock), decltype(control_mock)> estimation_and_control{estimation_mock,
                                                                                                                     control_mock,
                                                                                                                     period_in_ms};
};

TEST_F(EstimationAndControlTest, check_start_and_stop)
{
   EXPECT_CALL(estimation_mock, start());
   EXPECT_CALL(control_mock, start());
   estimation_and_control.start();

   EXPECT_CALL(estimation_mock, stop());
   EXPECT_CALL(control_mock, stop());
   estimation_and_control.stop();
}

TEST_F(EstimationAndControlTest, check_execute)
{
   EXPECT_CALL(estimation_mock, execute());
   EXPECT_CALL(control_mock, execute());
   estimation_and_control.run_once();
}

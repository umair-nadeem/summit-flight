#include "aeromight_control/Control.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "mocks/common/Logger.hpp"
#include "sys_time/ClockSource.hpp"

class AttitudeControllerMock
{
public:
   MOCK_METHOD(void, update, (const math::Vector3&, const math::Vector3&), ());
};

class RateControllerMock
{
public:
   MOCK_METHOD(void, update, (const float, const math::Vector3&, const math::Vector3&), ());
   MOCK_METHOD(void, reset, (), ());
};

class ControlTest : public testing::Test
{
protected:
   static constexpr float max_roll_rate_radps  = 3.5f;
   static constexpr float max_pitch_rate_radps = 3.5f;
   static constexpr float max_yaw_rate_radps   = 2.0f;
   static constexpr float max_tilt_angle_rad   = 30 * physics::constants::deg_to_rad;   // 30 degrees

   AttitudeControllerMock                                         attitude_controller_mock{};
   RateControllerMock                                             rate_controller_mock{};
   sys_time::ClockSource                                          sys_clock{};
   mocks::common::Logger                                          logger{"controlTest"};
   boundaries::SharedData<aeromight_boundaries::ControlHealth>    control_health_storage{};
   boundaries::SharedData<aeromight_boundaries::ControlSetpoints> control_setpoints_storage{};
   aeromight_control::StateEstimation                             state_estimation{};

   aeromight_control::Control<decltype(attitude_controller_mock),
                              decltype(rate_controller_mock),
                              decltype(sys_clock),
                              decltype(logger)>
       control{attitude_controller_mock,
               rate_controller_mock,
               control_health_storage,
               control_setpoints_storage,
               state_estimation,
               logger,
               max_roll_rate_radps,
               max_pitch_rate_radps,
               max_yaw_rate_radps,
               max_tilt_angle_rad};
};

TEST_F(ControlTest, check_start_and_stop)
{

   control.start();
}

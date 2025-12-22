#include "aeromight_control/Control.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "mocks/common/ClockSource.hpp"
#include "mocks/common/Logger.hpp"

class AttitudeControllerMock
{
public:
   MOCK_METHOD(math::Vector3, update, (const math::Vector3&, const math::Vector3&), ());
};

class RateControllerMock
{
public:
   static constexpr std::size_t num_axis = 3u;

   MOCK_METHOD(math::Vector3, update, (const math::Vector3&, const math::Vector3&, const float, const bool), ());
   MOCK_METHOD(void, set_saturation_status, ((const std::array<bool, 3u>&), (const std::array<bool, 3u>&)), ());
   MOCK_METHOD(void, reset, (), ());
};

class ControlAllocatorMock
{
public:
   MOCK_METHOD(aeromight_boundaries::ActuatorSetpoints, allocate, (const math::Vector4&), ());
   MOCK_METHOD(float, estimate_collective_thrust, (float), ());
   MOCK_METHOD((const std::array<bool, 3u>&), get_actuator_saturation_positive, (), ());
   MOCK_METHOD((const std::array<bool, 3u>&), get_actuator_saturation_negative, (), ());
};

class ControlTest : public testing::Test
{
protected:
   void provide_ticks(const uint32_t n)
   {
      uint32_t       current_ms_copy      = current_ms + 1u;
      const uint32_t ms_count_after_ticks = n + current_ms_copy;
      for (; current_ms_copy < ms_count_after_ticks; current_ms_copy++)
      {
         current_ms      = current_ms_copy;
         sys_clock.m_sec = current_ms;
         control.execute();
      }
   }

   static constexpr float max_roll_rate_radps  = 3.5f;
   static constexpr float max_pitch_rate_radps = 3.5f;
   static constexpr float max_yaw_rate_radps   = 2.0f;
   static constexpr float max_tilt_angle_rad   = 30 * physics::constants::deg_to_rad;   // 30 degrees
   static constexpr float lift_throttle        = 0.05f;

   AttitudeControllerMock                                               attitude_controller_mock{};
   RateControllerMock                                                   rate_controller_mock{};
   ControlAllocatorMock                                                 control_allocator_mock{};
   mocks::common::ClockSource                                           sys_clock{};
   mocks::common::Logger                                                logger{"controlTest"};
   boundaries::SharedData<aeromight_boundaries::ActuatorControl>        actuator_control_storage{};
   boundaries::SharedData<aeromight_boundaries::ControlHealth>          control_health_storage{};
   boundaries::SharedData<aeromight_boundaries::FlightControlSetpoints> flight_control_setpoints_storage{};
   aeromight_control::StateEstimation                                   state_estimation{};
   uint32_t                                                             current_ms{0};

   aeromight_control::Control<decltype(attitude_controller_mock),
                              decltype(rate_controller_mock),
                              decltype(control_allocator_mock),
                              decltype(sys_clock),
                              decltype(logger)>
       control{attitude_controller_mock,
               rate_controller_mock,
               control_allocator_mock,
               actuator_control_storage,
               control_health_storage,
               flight_control_setpoints_storage,
               state_estimation,
               logger,
               max_roll_rate_radps,
               max_pitch_rate_radps,
               max_yaw_rate_radps,
               max_tilt_angle_rad,
               lift_throttle};
};

TEST_F(ControlTest, do_nothing_until_activated)
{
   EXPECT_CALL(attitude_controller_mock, update).Times(0);
   EXPECT_CALL(rate_controller_mock, update).Times(0);

   provide_ticks(1u);
   const auto control_health = control_health_storage.get_latest();
   EXPECT_EQ(control_health.data.state, aeromight_boundaries::ControlState::inactive);
}

#pragma once

#include "aeromight_boundaries/ControlTaskEvents.hpp"
#include "aeromight_boundaries/StateEstimation.hpp"
#include "hardware_bindings.hpp"
#include "hw/timer/Timer.hpp"
#include "rtos/NotificationWaiter.hpp"
#include "utilities/pos_to_value.hpp"

extern "C"
{

   [[noreturn]] void control_task(void* const params);

}   // extern "C"

namespace controller
{

struct ControlTaskData
{
   using Channel = hw::timer::OutputChannel;

   // TIM1
   hw::timer::Timer master_pwm_timer{global_data.timer.tim1_config};
   // TIM4
   hw::timer::Timer slave_pwm_timer{global_data.timer.tim4_config};

   hw::timer::ChannelType master_pwm_timer_channels{utilities::pos_to_value(Channel::channel1) |
                                                    utilities::pos_to_value(Channel::channel2) |
                                                    utilities::pos_to_value(Channel::channel3)};

   hw::timer::ChannelType slave_pwm_timer_channels{utilities::pos_to_value(Channel::channel3)};

   struct MotorOutput
   {
      hw::timer::Timer& timer;
      Channel           channel;
   };

   std::array<MotorOutput, 4> motor_output_map{
       MotorOutput{master_pwm_timer, Channel::channel1},   // motor 1
       MotorOutput{master_pwm_timer, Channel::channel3},   // motor 2
       MotorOutput{slave_pwm_timer, Channel::channel3},    // motor 3
       MotorOutput{master_pwm_timer, Channel::channel2}    // motor 4
   };

   aeromight_boundaries::StateEstimation state_estimation{};

   rtos::NotificationWaiter<aeromight_boundaries::ControlTaskNotificationFlags> control_task_notification_waiter{};
};

extern ControlTaskData control_task_data;
}   // namespace controller

#pragma once

#include "aeromight_boundaries/ControlTaskEvents.hpp"
#include "aeromight_boundaries/StateEstimation.hpp"
#include "aeromight_boundaries/actuator.hpp"
#include "dshot/params.hpp"
#include "hardware_bindings.hpp"
#include "hw/dshot/Dshot.hpp"
#include "hw/dshot/DshotConfig.hpp"
#include "hw/pcb_component/Led.hpp"
#include "hw/timer/Timer.hpp"
#include "math/Vector4.hpp"
#include "rtos/NotificationWaiter.hpp"
#include "utilities/enum_to_bit_mask.hpp"

extern "C"
{

   [[noreturn]] void control_task(void* const params);

}   // extern "C"

namespace controller
{

struct ControlTaskData
{
   using Channel = hw::timer::OutputChannel;

   hw::timer::ChannelType master_pwm_timer_channels{utilities::enum_to_bit_mask<Channel::channel1>() |
                                                    utilities::enum_to_bit_mask<Channel::channel2>() |
                                                    utilities::enum_to_bit_mask<Channel::channel3>()};

   hw::timer::ChannelType slave_pwm_timer_channels{utilities::enum_to_bit_mask<Channel::channel3>()};

   // TIM1
   hw::timer::Timer master_pwm_timer{global_data.timer.tim1_config};
   // TIM4
   hw::timer::Timer slave_pwm_timer{global_data.timer.tim4_config};

   // dshot configs
   std::array<hw::dshot::DshotConfig, aeromight_boundaries::num_actuators>
       dshot_configs{
           {{DMA2, LL_DMA_STREAM_1,
             global_data.timer.tim1_config.timer_handle,
             LL_TIM_CHANNEL_CH1,
             &global_data.timer.tim1_config.timer_handle->CCR1},

            {DMA2, LL_DMA_STREAM_2,
             global_data.timer.tim1_config.timer_handle,
             LL_TIM_CHANNEL_CH2,
             &global_data.timer.tim1_config.timer_handle->CCR2},

            {DMA2, LL_DMA_STREAM_6,
             global_data.timer.tim1_config.timer_handle,
             LL_TIM_CHANNEL_CH3,
             &global_data.timer.tim1_config.timer_handle->CCR3},

            {DMA1, LL_DMA_STREAM_7,
             global_data.timer.tim4_config.timer_handle,
             LL_TIM_CHANNEL_CH3,
             &global_data.timer.tim4_config.timer_handle->CCR3}}};

   // dshot frame buffers
   std::array<std::array<uint16_t, dshot::frame_len>, aeromight_boundaries::num_actuators> dshot_buffers{};

   hw::dshot::Dshot<aeromight_boundaries::num_actuators> dshot{dshot_buffers,
                                                               dshot_configs,
                                                               static_cast<uint16_t>(global_data.timer.tim1_config.autoreload)};

   // motor output mapping
   math::Vec4<uint8_t> motor_output_map{
       0u,   // motor 1
       3u,   // motor 2
       1u,   // motor 3
       2u    // motor 4
   };

   hw::pcb_component::Led control_status_led{global_data.gpios.control_led, true};

   aeromight_boundaries::StateEstimation state_estimation{};

   rtos::NotificationWaiter control_task_notification_waiter{};
};

extern ControlTaskData control_task_data;
}   // namespace controller

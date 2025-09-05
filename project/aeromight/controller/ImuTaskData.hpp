#pragma once

#include <array>

#include "aeromight_boundaries/ImuTaskEvents.hpp"
#include "hardware_bindings.hpp"
#include "hw/gpio/DigitalOutput.hpp"
#include "hw/pcb_component/Enabler.hpp"
#include "hw/pcb_component/Led.hpp"
#include "hw/spi/SpiMasterWithDma.hpp"
#include "hw/timer/Timer.hpp"
#include "rtos/NotificationWaiter.hpp"
#include "rtos/Notifier.hpp"

extern "C"
{
   [[noreturn]] void imu_task(void* params);
}

namespace controller
{

struct ImuTaskData
{
   hw::pcb_component::Led blue_led{global_data.gpios.blue_led, true};

   // TIM3
   hw::timer::Timer imu_task_tick_timer{global_data.timer.tim3_config};

   // SPI1
   hw::gpio::DigitalOutput                                     spi1_chip_select_gpio{global_data.spi.spi1_chip_select, true};
   hw::pcb_component::Enabler<decltype(spi1_chip_select_gpio)> spi1_chip_select{spi1_chip_select_gpio};
   hw::spi::SpiMasterWithDma<decltype(spi1_chip_select)>       spi1_master{global_data.spi.spi1_config, spi1_chip_select};

   // task notification
   aeromight_boundaries::ImuNotificationFlags imu_task_tick_notification{aeromight_boundaries::pos_to_value(aeromight_boundaries::ImuTaskEvents::tick)};
   aeromight_boundaries::ImuNotificationFlags imu_rx_complete_notification{aeromight_boundaries::pos_to_value(aeromight_boundaries::ImuTaskEvents::rx_complete)};

   rtos::NotificationWaiter<aeromight_boundaries::ImuNotificationFlags> imu_task_notification_waiter{};
   rtos::Notifier<aeromight_boundaries::ImuNotificationFlags>           imu_task_tick_notifier_from_isr{imu_task_tick_notification};
   rtos::Notifier<aeromight_boundaries::ImuNotificationFlags>           imu_task_rx_complete_notifier_from_isr{imu_rx_complete_notification};
};

extern ImuTaskData imu_task_data;

}   // namespace controller

#pragma once

#include "aeromight_boundaries/BarometerTaskEvents.hpp"
#include "hardware_bindings.hpp"
#include "hw/i2c/I2c.hpp"
#include "rtos/NotificationWaiter.hpp"
#include "rtos/Notifier.hpp"

void i2c1_receive_complete_callback(void*);
extern "C"
{
   [[noreturn]] void barometer_task(void* params);
}

namespace controller
{

struct BarometerTaskData
{
   // I2C1
   hw::i2c::I2c i2c_driver{global_data.i2c.i2c1_config};

   // task notification
   aeromight_boundaries::BarometerNotificationFlags barometer_rx_complete_notification{aeromight_boundaries::pos_to_value(aeromight_boundaries::BarometerTaskEvents::rx_complete)};

   rtos::NotificationWaiter<aeromight_boundaries::BarometerNotificationFlags> barometer_task_notification_waiter{};
   rtos::Notifier<aeromight_boundaries::BarometerNotificationFlags>           barometer_task_rx_complete_notifier_from_isr{barometer_rx_complete_notification};
};

extern BarometerTaskData barometer_task_data;

}   // namespace controller

#pragma once

#include "aeromight_boundaries/BarometerTaskEvents.hpp"
#include "hardware_bindings.hpp"
#include "hw/i2c/I2c.hpp"
#include "rtos/NotificationWaiter.hpp"
#include "rtos/Notifier.hpp"
#include "utilities/enum_to_bit_mask.hpp"

void i2c1_receive_complete_callback(void*);
extern "C"
{
   [[noreturn]] void barometer_task(void* const params);
}   // extern "C"

namespace controller
{

struct BarometerTaskData
{
   // I2C1
   hw::i2c::I2c i2c_driver{global_data.i2c.i2c1_config};

   // task notification
   static constexpr auto    event_tick_bit_mask        = utilities::enum_to_bit_mask<aeromight_boundaries::BarometerTaskEvents::tick>();
   static constexpr auto    event_rx_complete_bit_mask = utilities::enum_to_bit_mask<aeromight_boundaries::BarometerTaskEvents::rx_complete>();
   rtos::NotificationWaiter barometer_task_notification_waiter{};
   rtos::Notifier           barometer_task_tick_notifier_from_task{event_tick_bit_mask};
   rtos::Notifier           barometer_task_rx_complete_notifier_from_isr{event_rx_complete_bit_mask};
};

extern BarometerTaskData barometer_task_data;

}   // namespace controller

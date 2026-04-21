#pragma once

#include <array>

#include "aeromight_boundaries/ImuTaskEvents.hpp"
#include "hardware_bindings.hpp"
#include "hw/gpio/DigitalOutput.hpp"
#include "hw/pcb_component/Enabler.hpp"
#include "hw/spi/SpiMasterWithDma.hpp"
#include "hw/timer/Timer.hpp"
#include "imu_sensor/ImuSensorStatus.hpp"
#include "imu_sensor/RawImuSensorData.hpp"
#include "rtos/NotificationWaiter.hpp"
#include "rtos/Notifier.hpp"
#include "utilities/enum_to_bit_mask.hpp"

extern "C"
{
   [[noreturn]] void imu_task(void* const params);
}

namespace controller
{

struct ImuTaskData
{
   // TIM3
   hw::timer::Timer imu_task_tick_timer{global_data.timer.tim3_config};

   // SPI1
   hw::gpio::DigitalOutput                                     spi1_chip_select_gpio{global_data.spi.spi1_chip_select, true};
   hw::pcb_component::Enabler<decltype(spi1_chip_select_gpio)> spi1_chip_select{spi1_chip_select_gpio};
   hw::spi::SpiMasterWithDma<decltype(spi1_chip_select)>       spi1_master{global_data.spi.spi1_config, spi1_chip_select};

   // task notification
   static constexpr auto event_tick_bit_mask        = utilities::enum_to_bit_mask<aeromight_boundaries::ImuTaskEvents::tick>();
   static constexpr auto event_rx_complete_bit_mask = utilities::enum_to_bit_mask<aeromight_boundaries::ImuTaskEvents::rx_complete>();
   static constexpr auto event_calibrate_bit_mask   = utilities::enum_to_bit_mask<aeromight_boundaries::ImuTaskEvents::calibrate>();

   rtos::NotificationWaiter imu_task_notification_waiter{};
   rtos::Notifier           imu_task_tick_notifier_from_isr{event_tick_bit_mask};
   rtos::Notifier           imu_task_rx_complete_notifier_from_isr{event_rx_complete_bit_mask};

   // sensors
   imu_sensor::RawImuSensorData imu_sensor_data{};
   imu_sensor::ImuSensorStatus  imu_sensor_status{};
};

extern ImuTaskData imu_task_data;

}   // namespace controller

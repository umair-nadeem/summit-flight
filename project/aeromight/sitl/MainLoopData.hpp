#pragma once

#include "aeromight_boundaries/ControlTaskEvents.hpp"
#include "aeromight_boundaries/HealthSummary.hpp"
#include "aeromight_boundaries/ImuTaskEvents.hpp"
#include "aeromight_boundaries/StateEstimation.hpp"
#include "boundaries/SharedData.hpp"
#include "imu_sensor/ImuSensorStatus.hpp"
#include "imu_sensor/RawImuSensorData.hpp"
#include "math/Vector4.hpp"
#include "power/battery/Battery.hpp"
#include "power/battery/BatteryPercentageModel.hpp"
#include "rtos/Notification.hpp"
#include "rtos/Queue.hpp"
#include "sitl//pcb_component/Led.hpp"
#include "sitl/actuator/ActuatorControl.hpp"
#include "sitl/hw/Adc.hpp"
#include "utilities/enum_to_bit_mask.hpp"

namespace sitl
{

void run_main_loop(void* const params);

struct MainLoopData
{
   static constexpr float battery_voltage_calibration_constant = 0.988f;

   // imu
   imu_sensor::RawImuSensorData imu_sensor_data{};
   imu_sensor::ImuSensorStatus  imu_sensor_status{};

   // estimation
   aeromight_boundaries::StateEstimation state_estimation{};

   // control
   sitl::pcb_component::Led            control_led{};
   sitl::actuator::ActuatorControl<4u> actuator_control{};

   // motor output mapping
   math::Vec4<uint8_t> motor_output_map{
       0u,   // motor 1
       1u,   // motor 2
       2u,   // motor 3
       3u    // motor 4
   };

   // health monitoring
   sitl::hw::Adc adc{};

   power::battery::VoltageSenseConfig battery_voltage_sense{};

   power::battery::PercentageModelLipo4S percentage_convertor{};

   power::battery::Battery<decltype(adc),
                           decltype(percentage_convertor)>
       battery{adc,
               percentage_convertor,
               battery_voltage_sense,
               battery_voltage_calibration_constant};

   rtos::Queue<aeromight_boundaries::HealthSummary> health_summary_queue{10u};

   // system manager
   sitl::pcb_component::Led status_led{};
   rtos::Notification       control_task_start_notifier{utilities::enum_to_bit_mask<aeromight_boundaries::ControlTaskEvents::start>()};
   rtos::Notification       imu_task_calibrate_notifier{utilities::enum_to_bit_mask<aeromight_boundaries::ImuTaskEvents::calibrate>()};
};

extern MainLoopData main_loop_data;

}   // namespace sitl

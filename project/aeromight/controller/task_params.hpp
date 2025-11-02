#pragma once

namespace controller::task
{

// configMAX_PRIORITIES -> 10
static constexpr uint32_t system_core_clock = 100'000'000u;

// list of tasks in decreasing priority
// IMU Task
static constexpr const char* const imu_task_name{"imu_sensor"};
static constexpr uint32_t          imu_task_stack_size_in_bytes{4096u};
static constexpr uint32_t          imu_task_stack_depth_in_words{imu_task_stack_size_in_bytes / sizeof(uint32_t)};
static constexpr uint32_t          imu_task_priority{8u};
static constexpr uint32_t          imu_task_period_in_ms{4u};

// Control Task
static constexpr const char* const control_task_name{"estm_ctrl"};
static constexpr uint32_t          control_task_stack_size_in_bytes{4096u};
static constexpr uint32_t          control_task_stack_depth_in_words{control_task_stack_size_in_bytes / sizeof(uint32_t)};
static constexpr uint32_t          control_task_priority{8u};
static constexpr uint32_t          control_task_period_in_ms{4u};

// Flight Manager Task
static constexpr const char* const flight_manager_task_name{"flight_mngr"};
static constexpr uint32_t          flight_manager_task_stack_size_in_bytes{4096u};
static constexpr uint32_t          flight_manager_task_stack_depth_in_words{flight_manager_task_stack_size_in_bytes / sizeof(uint32_t)};
static constexpr uint32_t          flight_manager_task_priority{6u};
static constexpr uint32_t          flight_manager_task_period_in_ms{20u};

// Health Monitoring Task
static constexpr const char* const health_monitoring_task_name{"health_mon"};
static constexpr uint32_t          health_monitoring_task_stack_size_in_bytes{2048u};
static constexpr uint32_t          health_monitoring_task_stack_depth_in_words{health_monitoring_task_stack_size_in_bytes / sizeof(uint32_t)};
static constexpr uint32_t          health_monitoring_task_priority{4u};
static constexpr uint32_t          health_monitoring_task_period_in_ms{40u};

// Barometer Task
static constexpr const char* const barometer_task_name{"baro_sensor"};
static constexpr uint32_t          barometer_task_stack_size_in_bytes{4096u};
static constexpr uint32_t          barometer_task_stack_depth_in_words{barometer_task_stack_size_in_bytes / sizeof(uint32_t)};
static constexpr uint32_t          barometer_task_priority{4u};
static constexpr uint32_t          barometer_task_period_in_ms{40u};

// Logging Task
static constexpr const char* const logging_task_name{"logging"};
static constexpr uint32_t          logging_task_stack_size_in_bytes{1024u};
static constexpr uint32_t          logging_task_stack_depth_in_words{logging_task_stack_size_in_bytes / sizeof(uint32_t)};
static constexpr uint32_t          logging_task_priority{2u};

}   // namespace controller::task

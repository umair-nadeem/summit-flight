#pragma once

namespace controller::task
{

// configMAX_PRIORITIES -> 10
static constexpr uint32_t system_core_clock = 100'000'000u;

// IMU Task
static constexpr const char* const imu_task_name{"imu_task"};
static constexpr std::size_t       imu_task_stack_size_in_bytes{4096u};
static constexpr std::size_t       imu_task_stack_depth_in_words{imu_task_stack_size_in_bytes / sizeof(uint32_t)};
static constexpr std::size_t       imu_task_priority{8u};
static constexpr std::size_t       imu_task_period_in_ms{4u};

// Control Task
static constexpr const char* const control_task_name{"ctrl_task"};
static constexpr std::size_t       control_task_stack_size_in_bytes{4096u};
static constexpr std::size_t       control_task_stack_depth_in_words{control_task_stack_size_in_bytes / sizeof(uint32_t)};
static constexpr std::size_t       control_task_priority{7u};
static constexpr std::size_t       control_task_period_in_ms{8u};

// Barometer Task
static constexpr const char* const barometer_task_name{"baro_task"};
static constexpr std::size_t       barometer_task_stack_size_in_bytes{4096u};
static constexpr std::size_t       barometer_task_stack_depth_in_words{barometer_task_stack_size_in_bytes / sizeof(uint32_t)};
static constexpr std::size_t       barometer_task_priority{4u};
static constexpr std::size_t       barometer_task_period_in_ms{40u};

// Logging Task
static constexpr const char* const logging_task_name{"logging_task"};
static constexpr std::size_t       logging_task_stack_size_in_bytes{1024u};
static constexpr std::size_t       logging_task_stack_depth_in_words{logging_task_stack_size_in_bytes / sizeof(uint32_t)};
static constexpr std::size_t       logging_task_priority{2u};

}   // namespace controller::task

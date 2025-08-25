#pragma once

namespace controller::task
{

// IMU Task
static constexpr const char* const imu_task_name{"imu_task"};
static constexpr std::size_t       imu_task_stack_size_in_bytes{4096u};
static constexpr std::size_t       imu_task_stack_depth_in_words{imu_task_stack_size_in_bytes / sizeof(uint32_t)};
static constexpr std::size_t       imu_task_priority{4u};
static constexpr std::size_t       imu_task_period_in_ms{4u};

// Logging Task
static constexpr const char* const logging_task_name{"logging_task"};
static constexpr std::size_t       logging_task_stack_size_in_bytes{1024u};
static constexpr std::size_t       logging_task_stack_depth_in_words{logging_task_stack_size_in_bytes / sizeof(uint32_t)};
static constexpr std::size_t       logging_task_priority{2u};

}   // namespace controller::task

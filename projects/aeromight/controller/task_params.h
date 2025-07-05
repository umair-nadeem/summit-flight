#pragma once

namespace controller
{

// Sensor Acquistion Task
static constexpr char const * const sensor_acq_task_name{"snsr_acq_task"};
static constexpr size_t sensor_acq_task_stack_size_in_bytes{4096u};
static constexpr size_t sensor_acq_task_stack_depth_in_words{sensor_acq_task_stack_size_in_bytes / sizeof(uint32_t)};
static constexpr size_t sensor_acq_task_priority{4u};
static constexpr size_t sensor_acq_task_period_in_ms{4u};

}  // namespace controller


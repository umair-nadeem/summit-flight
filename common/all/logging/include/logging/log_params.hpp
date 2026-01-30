#pragma once

#include <array>

namespace logging::params
{

static constexpr std::size_t max_log_len         = 192u;
static constexpr std::size_t logging_queue_depth = 32u;
static constexpr std::size_t max_logger_name_len = 10u;

using LogBuffer = std::array<char, max_log_len>;

}   // namespace logging::params

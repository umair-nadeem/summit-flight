#pragma once

namespace logging::params
{

static constexpr std::size_t max_log_len         = 64u;
static constexpr std::size_t logging_queue_len   = 20u;
static constexpr std::size_t max_logger_name_len = 10u;

using LogBuffer = std::array<char, max_log_len>;

}   // namespace logging::params

#pragma once

namespace bmp390
{

struct Bmp390Params
{
   uint8_t  read_failures_limit     = 5u;
   uint8_t  max_recovery_attempts   = 3u;
   uint8_t  execution_period_ms     = 40u;
   uint32_t receive_wait_timeout_ms = 2u * execution_period_ms;
};

}   // namespace bmp390

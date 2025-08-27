#pragma once

#include <atomic>

#include "interfaces/IClockSource.hpp"

namespace sys_time
{

struct ClockSource
{
   static_assert(std::atomic<uint32_t>::is_always_lock_free, "atomic<uint32_t> must be lock-free on this platform");

   static void init()
   {
      clock_ms.store(0, std::memory_order_relaxed);
   }

   static void tick_clock()
   {
      clock_ms.fetch_add(1u, std::memory_order_relaxed);
   }

   static uint32_t now_ms() noexcept
   {
      return clock_ms.load(std::memory_order_acquire);
   }

   static uint32_t now_s() noexcept
   {
      return (now_ms() / 1000u);
   }

   static uint32_t elapsed_since(const uint32_t start_ms)
   {
      return now_ms() - start_ms;
   }

   static bool has_elapsed(const uint32_t start_ms, const uint32_t delta_ms)
   {
      return elapsed_since(start_ms) >= delta_ms;
   }

   static std::atomic<uint32_t> clock_ms;
};

static_assert(interfaces::IClockSource<ClockSource>);

}   // namespace sys_time

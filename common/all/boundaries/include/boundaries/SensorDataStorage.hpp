#pragma once

#include "interfaces/IClockSource.hpp"

namespace boundaries
{

template <typename T, interfaces::IClockSource ClockSource>
struct SensorDataStorage
{
   static_assert(std::atomic<uint8_t>::is_always_lock_free, "atomic<uint8_t> must be lock-free on this platform");
   static_assert(std::is_trivially_copyable_v<T>, "SensorDataStorage<T>: T must be trivially copyable");

   struct Sample
   {
      T        data;
      uint64_t timestamp_us{};
   };

   std::array<Sample, 2u> samples{};
   std::atomic<uint8_t>   current_index{0};

   inline void update_latest(const T& new_data)
   {
      uint8_t next_index               = static_cast<uint8_t>((current_index.load(std::memory_order_relaxed) + 1u) % 2u);
      samples[next_index].data         = new_data;
      samples[next_index].timestamp_us = ClockSource::now_us();
      current_index.store(next_index, std::memory_order_release);
   }

   inline Sample get_latest() const
   {
      return samples[current_index.load(std::memory_order_acquire)];
   }
};

}   // namespace boundaries

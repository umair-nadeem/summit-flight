#pragma once

#include <array>
#include <atomic>

namespace boundaries
{

template <typename T>
struct SharedData
{
   static_assert(std::atomic<uint32_t>::is_always_lock_free, "atomic<uint32_t> must be lock-free on this platform");
   static_assert(std::is_trivially_copyable_v<T>, "SharedData<T>: T must be trivially copyable");

   struct Sample
   {
      T        data;
      uint32_t timestamp_ms{};
   };

   static constexpr uint32_t num_buffered_samples{3u};

   std::array<Sample, num_buffered_samples> samples{};
   std::atomic<uint32_t>                    read_index{0};    // Reader always reads from this buffer, writer updates it -> atomic
   uint32_t                                 write_index{1};   // Writer always writes to this buffer, only writer touches it -> non-atomic
   uint32_t                                 spare_index{2};   // Spare buffer waiting to become write buf, only writer touches it -> non-atomic

   inline void update_latest(const T& new_data, const auto ts_ms) noexcept
   {
      samples[write_index].data         = new_data;
      samples[write_index].timestamp_ms = ts_ms;

      // Rotate indices: spare→read, write→spare, read→write
      const uint32_t old_read = read_index.load(std::memory_order_relaxed);

      read_index.store(write_index, std::memory_order_release);
      write_index = spare_index;
      spare_index = old_read;
   }

   inline Sample get_latest() const
   {
      return samples[read_index.load(std::memory_order_acquire)];
   }
};

}   // namespace boundaries

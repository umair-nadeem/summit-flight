#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>

namespace rtos
{

class Notification
{
public:
   void notify(const uint32_t bits = 1u)
   {
      {
         std::lock_guard<std::mutex> lock(m_mutex);
         m_bits |= bits;
      }
      m_cv.notify_one();
   }

   void notify_from_isr(const uint32_t bits = 1u)
   {
      notify(bits);
   }

   uint32_t wait(const uint32_t timeout_ms)
   {
      std::unique_lock<std::mutex> lock(m_mutex);

      if (timeout_ms == 0u)
      {
         uint32_t bits = m_bits;
         m_bits        = 0u;
         return bits;
      }

      const auto timeout = std::chrono::milliseconds(timeout_ms);

      const bool woke = m_cv.wait_for(lock, timeout, [this]()
                                      { return m_bits != 0u; });

      if (!woke)
      {
         return 0u;
      }

      uint32_t bits = m_bits;
      m_bits        = 0u;
      return bits;
   }

private:
   std::mutex              m_mutex;
   std::condition_variable m_cv;
   uint32_t                m_bits{0u};
};

}   // namespace rtos

#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>

#include "error/error_handler.hpp"

namespace rtos
{

class Notification
{
public:
   explicit Notification(const types::EventBitsType bits_to_notify)
       : m_bits_to_notify{bits_to_notify}
   {
      error::verify(bits_to_notify > 0);
   }

   void notify(const types::EventBitsType bits)
   {
      {
         std::lock_guard<std::mutex> lock(m_mutex);
         m_bits |= bits;
      }
      m_cv.notify_one();
   }

   void notify()
   {
      notify(m_bits_to_notify);
   }

   void notify_from_isr()
   {
      notify(m_bits_to_notify);
   }

   types::EventBitsType wait(const uint32_t timeout_ms)
   {
      std::unique_lock<std::mutex> lock(m_mutex);

      if (timeout_ms == 0u)
      {
         const types::EventBitsType bits = m_bits;
         m_bits                          = 0u;
         return bits;
      }

      const auto timeout = std::chrono::milliseconds(timeout_ms);

      const bool woke = m_cv.wait_for(lock, timeout, [this]()
                                      { return m_bits != 0u; });

      if (!woke)
      {
         return 0u;
      }

      const types::EventBitsType bits = m_bits;
      m_bits                          = 0u;
      return bits;
   }

private:
   const types::EventBitsType m_bits_to_notify;
   std::mutex                 m_mutex{};
   std::condition_variable    m_cv{};
   types::EventBitsType       m_bits{0u};
};

}   // namespace rtos

#pragma once

#include <condition_variable>
#include <mutex>
#include <optional>
#include <queue>

namespace rtos
{

template <typename T>
class Queue
{
public:
   explicit Queue(const std::size_t capacity)
       : m_capacity{capacity}
   {
   }

   void send_blocking(const T& value)
   {
      std::unique_lock<std::mutex> lock(m_mutex);

      m_not_full.wait(lock, [this]()
                      { return m_queue.size() < m_capacity; });

      m_queue.push(value);

      lock.unlock();
      m_not_empty.notify_one();
   }

   bool send_if_possible(const T& value)
   {
      std::lock_guard<std::mutex> lock(m_mutex);

      if (m_queue.size() >= m_capacity)
      {
         return false;
      }

      m_queue.push(value);
      m_not_empty.notify_one();
      return true;
   }

   bool send_from_isr(const T& value)
   {
      return send_if_possible(value);
   }

   T receive_blocking()
   {
      std::unique_lock<std::mutex> lock(m_mutex);

      m_not_empty.wait(lock, [this]()
                       { return !m_queue.empty(); });

      T value = m_queue.front();
      m_queue.pop();

      lock.unlock();
      m_not_full.notify_one();

      return value;
   }

   std::optional<T> receive_if_available()
   {
      std::lock_guard<std::mutex> lock(m_mutex);

      if (m_queue.empty())
      {
         return std::nullopt;
      }

      T value = m_queue.front();
      m_queue.pop();

      m_not_full.notify_one();
      return value;
   }

   std::optional<T> receive_latest()
   {
      return receive_if_available();
   }

private:
   const std::size_t       m_capacity;
   std::mutex              m_mutex{};
   std::condition_variable m_not_empty{};
   std::condition_variable m_not_full{};
   std::queue<T>           m_queue{};
};

}   // namespace rtos

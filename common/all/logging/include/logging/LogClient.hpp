#pragma once

#include <cstring>

#include <printf/printf.h>

#include "logging/log_params.h"

namespace logging
{

template <typename QueueSender>
class LogClient
{
public:
   explicit LogClient(QueueSender& queue_sender, const char* name)
       : m_queue_sender{queue_sender}
   {
      strncpy(m_logger_name, name, params::max_logger_name_len);
      m_logger_name[params::max_logger_name_len] = '\0';
   }

   void enable()
   {
      m_enabled = true;
   }

   void print(const char* msg)
   {
      printf("%s", msg);
   }

   template <typename... Args>
   void printf(const char* fmt, Args&&... args)
   {
      if (m_enabled)
      {
         m_total_chars_written = 0;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"

         auto chars_written_for_name = snprintf_(m_log_buffer.data(), params::max_log_len, "%s: ", m_logger_name);

         if (chars_written_for_name >= 0)
         {
            m_total_chars_written = std::min(params::max_log_len, static_cast<size_t>(chars_written_for_name));

            auto chars_written_for_log = snprintf_(m_log_buffer.data() + m_total_chars_written, params::max_log_len - m_total_chars_written, fmt, args...);

            if (chars_written_for_log >= 0)
            {
               m_total_chars_written = std::min(params::max_log_len, m_total_chars_written + static_cast<size_t>(chars_written_for_log));
               send_to_queue();
            }
         }

#pragma GCC diagnostic pop
      }
   }

private:
   void send_to_queue()
   {
      const auto linefeed_position    = std::min(static_cast<uint32_t>(m_log_buffer.size() - delimiter_length), static_cast<uint32_t>(m_total_chars_written));
      m_log_buffer[linefeed_position] = '\n';
      memset(m_log_buffer.data() + linefeed_position + 1, '\0', m_log_buffer.size() - (linefeed_position + 1));
      [[maybe_unused]] bool result = m_queue_sender.send_if_possible(m_log_buffer);
   }

   static constexpr size_t delimiter_length = 2U;   // two characters, \n and \0
   static_assert(params::max_log_len >= params::max_logger_name_len + delimiter_length);

   QueueSender&      m_queue_sender;
   char              m_logger_name[params::max_logger_name_len + 1u];
   params::LogBuffer m_log_buffer{};
   bool              m_enabled{false};
   std::size_t       m_total_chars_written{};
};

}   // namespace logging

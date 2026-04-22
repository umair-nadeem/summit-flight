#pragma once

#include <cstdio>
#include <cstring>

namespace logging
{

class Logger
{
public:
   explicit Logger(const char* const name)
   {
      strncpy(m_logger_name, name, max_logger_name_len);
      m_logger_name[max_logger_name_len] = '\0';
   }

   void enable()
   {
      m_enabled = true;
   }

   void print(const char* text) const
   {
      if (m_enabled)
      {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
         std::printf("%s: %s\n", m_logger_name, text);
         std::fflush(stdout);
#pragma GCC diagnostic pop
      }
   }

   template <typename... Args>
   void printf(const char* fmt, Args&&... args) const
   {
      if (m_enabled)
      {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
         std::printf("%s: ", m_logger_name);
         std::printf(fmt, std::forward<Args>(args)...);
         std::printf("\n");
         std::fflush(stdout);
#pragma GCC diagnostic pop
      }
   }

private:
   static constexpr std::size_t max_logger_name_len = 10u;

   char m_logger_name[max_logger_name_len + 1u];
   bool m_enabled{false};
};

}   // namespace logging

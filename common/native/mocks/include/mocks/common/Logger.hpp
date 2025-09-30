#pragma once

#include <iostream>

#include <gmock/gmock.h>

namespace mocks::common
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
         std::cout << m_logger_name << ": " << text << "\n";
      }
   }

   template <typename... Args>
   void printf(const char* fmt, Args&&... args) const
   {
      if (m_enabled)
      {
         std::cout << m_logger_name << ": " << fmt;                  // print the "format" text as-is
         ((std::cout << ... << (std::cout << " ", args)) << '\n');   // fold expression to print remaining args
      }
   }

private:
   static constexpr std::size_t max_logger_name_len = 10u;

   char m_logger_name[max_logger_name_len + 1u];
   bool m_enabled{false};
};

}   // namespace mocks::common

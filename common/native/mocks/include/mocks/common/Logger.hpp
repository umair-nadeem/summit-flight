#pragma once

#include <iostream>

#include <gmock/gmock.h>

namespace mocks::common
{

class Logger
{
public:
   void enable()
   {
      m_enabled = true;
   }

   void print(const char* text) const
   {
      if (m_enabled)
      {
         std::cout << text << "\n";
      }
   }

private:
   bool m_enabled{false};
};

}   // namespace mocks::common

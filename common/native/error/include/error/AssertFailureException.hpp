#pragma once

#include <stdexcept>

namespace error
{

class AssertFailureException : public std::runtime_error
{
public:
   explicit AssertFailureException(const char* arg)
       : std::runtime_error(arg)
   {
   }
};

}   // namespace error

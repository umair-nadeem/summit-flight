#include "error/error_handler.hpp"

#include "error/AssertFailureException.hpp"

namespace error
{

void verify(const bool condition)
{
   if (!condition)
   {
      throw AssertFailureException("error: verify failure");
   }
}

void stop_operation()
{
   throw AssertFailureException("error: fatal condition");
}

}   // namespace error

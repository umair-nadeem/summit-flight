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

}   // namespace error

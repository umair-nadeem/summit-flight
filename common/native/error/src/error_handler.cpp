#include "error/error_handler.h"

#include "error/AssertFailureException.h"

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

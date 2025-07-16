#include "error/error_handler.h"

#include "error/AssertFailureException.h"

namespace error
{

void assert(const bool condition)
{
   if (!condition)
   {
      throw AssertFailureException("error: assert failure");
   }
}

}   // namespace error

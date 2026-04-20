#include "error/error_handler.hpp"

extern "C"
{
   void putchar_([[maybe_unused]] char c)
   {
      error::stop_operation();
   }
}   // extern "C"

int main()
{

   return -1;
}

#include "stdlib.h"

int main()
{
   int x = -35;
   int y = 10;

   x = x + y;

   for (size_t i=0; i<5; i++)
   {
      x += i;
   }

   return 0;
}

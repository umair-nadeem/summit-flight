#include <stdlib.h>
#include <cstdio>

#include "main.h"
#include "FreeRTOS.h"

static constexpr uint32_t value = TICK_TYPE_WIDTH_32_BITS;

extern "C"
{

void start()
{
   size_t clock_value = SystemCoreClock;

   printf("value of clock is %u", clock_value);

   static constexpr size_t blocking_delay = 500u;
   static constexpr size_t min_delay_multiplier = 1u;
   static constexpr size_t max_delay_multiplier = 10;

   size_t delay_multiplier = min_delay_multiplier;
   bool increasing_delay = true;

   while (1)
   {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      HAL_Delay(delay_multiplier * blocking_delay);

      increasing_delay ? ++delay_multiplier : --delay_multiplier;

      // max limit reached, oscillate downwards
      if (delay_multiplier >= max_delay_multiplier)
      {
         delay_multiplier = max_delay_multiplier;
         increasing_delay = false;
      }

      if (delay_multiplier <= min_delay_multiplier)
      {
         delay_multiplier = min_delay_multiplier;
         increasing_delay = true;
      }
   }

}

}

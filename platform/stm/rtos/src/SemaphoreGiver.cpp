#include "rtos/SemaphoreGiver.hpp"

namespace rtos
{

void SemaphoreGiver::give()
{
   const BaseType_t result = xSemaphoreGive(m_handle);
   error::freertos_assert(result == pdPASS);
}

bool SemaphoreGiver::give_from_isr()
{
   BaseType_t       task_woken = pdFALSE;
   const BaseType_t result     = xSemaphoreGiveFromISR(m_handle, &task_woken);
   if (result == errQUEUE_FULL)
   {
      return false;
   }

   error::freertos_assert(result == pdPASS);
   portYIELD_FROM_ISR(task_woken);
   return true;
}

}   // namespace rtos

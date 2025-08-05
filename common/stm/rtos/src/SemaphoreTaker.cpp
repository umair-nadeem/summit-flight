#include "rtos/SemaphoreTaker.hpp"

namespace rtos
{

void SemaphoreTaker::take()
{
   BaseType_t result;
   do
   {
      result = xSemaphoreTake(m_handle, portMAX_DELAY);
   } while (result == errQUEUE_EMPTY);
   error::freertos_assert(result == pdPASS);
}

bool SemaphoreTaker::take_if_possible()
{
   const BaseType_t result = xSemaphoreTake(m_handle, 0);
   if (result == errQUEUE_EMPTY)
   {
      return false;
   }

   error::freertos_assert(result == pdPASS);
   return true;
}

bool SemaphoreTaker::take_from_isr(const bool higher_priority_task_woken)
{
   BaseType_t       task_woken = (higher_priority_task_woken ? pdTRUE : pdFALSE);
   const BaseType_t result     = xSemaphoreTakeFromISR(m_handle, &task_woken);
   if (result == errQUEUE_EMPTY)
   {
      return false;
   }

   error::freertos_assert(result == pdPASS);
   portYIELD_FROM_ISR(task_woken);
   return true;
}

}   // namespace rtos

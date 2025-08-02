#include "error/freertos_errors.h"

#include <cassert>
#include <string.h>

#include "error/error_record.h"

namespace error
{

void freertos_assert(const bool result, const std::source_location location)
{
   if (!result)
   {
      error::FreertosAssertFailureRecord record{};

      const auto* pc = __builtin_return_address(0);

      strncpy(record.file, location.file_name(), sizeof(record.file) - 1u);
      record.line           = static_cast<int32_t>(location.line());
      record.caller_address = reinterpret_cast<uint32_t>(pc);

      add_error_record(record);
   }
}

}   // namespace error

extern "C"
{
   void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
   {
      error::FreertosStackOverflowRecord record{};

      const auto* pc = __builtin_return_address(0);

      strncpy(record.name, pcTaskName, sizeof(record.name) - 1u);
      record.handle         = reinterpret_cast<uintptr_t>(xTask);
      record.caller_address = reinterpret_cast<uint32_t>(pc);

      add_error_record(record);
   }

   void freertos_config_assert(const char* expr, const char* file, int line)
   {
      error::FreertosAssertFailureRecord record{};

      const auto* pc = __builtin_return_address(0);

      strncpy(record.expr, expr, sizeof(record.expr) - 1u);
      strncpy(record.file, file, sizeof(record.file) - 1u);
      record.line           = line;
      record.caller_address = reinterpret_cast<uint32_t>(pc);

      add_error_record(record);
   }
}   // extern "C"

#include <string.h>

#include "error/error_record.h"

extern "C"
{
   void hal_error_handler(const uint32_t pc)
   {
      add_error_record(error::HalErrorRecord{.caller_address = pc});
   }

   void hal_failed_assert_handler(const char* file, const uint32_t line, const uint32_t pc)
   {
      error::HalAssertFailureRecord record{};

      strncpy(record.file, file, sizeof(record.file) - 1u);
      record.line           = line;
      record.caller_address = pc;

      add_error_record(record);
   }

}   // extern "C"

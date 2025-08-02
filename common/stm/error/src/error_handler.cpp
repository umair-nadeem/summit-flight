
#include "error/error_handler.h"

#include <string.h>

#include "error/error_record.h"

namespace error
{

void verify(const bool condition, const std::source_location location)
{
   if (!condition)
   {
      GeneralFailureRecord record{};
      strncpy(record.file_name, location.file_name(), sizeof(record.file_name) - 1u);
      strncpy(record.function_name, location.function_name(), sizeof(record.function_name) - 1u);
      record.line = location.line();

      add_error_record(record);
   }
}

void stop_operation(const std::source_location location)
{
   GeneralFailureRecord record{};
   strncpy(record.file_name, location.file_name(), sizeof(record.file_name) - 1u);
   strncpy(record.function_name, location.function_name(), sizeof(record.function_name) - 1u);
   record.line = location.line();

   add_error_record(record);
}

}   // namespace error

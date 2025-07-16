
#include "error/error_handler.h"

#include <string.h>

#include "error/error_record.h"

namespace error
{

void assert(const bool condition, const std::source_location location)
{
   if (!condition)
   {
      GeneralAssertRecord record{};
      strncpy(record.file_name, location.file_name(), sizeof(record.file_name) - 1u);
      strncpy(record.function_name, location.function_name(), sizeof(record.function_name) - 1u);
      record.line = location.line();

      add_error_record(record);
   }
}

}   // namespace error

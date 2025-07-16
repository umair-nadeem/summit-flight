#include "error/error_record.h"

namespace error
{

MasterErrorRecord master_error_record{};

void add_error_record(const MasterErrorRecord& error_record)
{
   master_error_record = error_record;

   __asm("BKPT #0");

   __disable_irq();

   while (true)
   {
   }
}

bool is_error_record_found()
{
   return std::holds_alternative<std::monostate>(master_error_record);
}

}   // namespace error

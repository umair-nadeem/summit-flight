#pragma once

namespace error
{

struct GeneralAssertRecord
{
   char     file_name[32];
   char     function_name[32];
   uint32_t line;
};

}   // namespace error

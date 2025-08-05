#pragma once

namespace error
{

struct HalErrorRecord
{
   uint32_t caller_address;
};

struct HalAssertFailureRecord
{
   char     file[32];
   uint32_t line;
   uint32_t caller_address;
};

}   // namespace error

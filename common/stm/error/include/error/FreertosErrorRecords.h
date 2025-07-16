#pragma once

#include "FreeRTOS.h"
#include "task.h"

namespace error
{

struct FreertosStackOverflowRecord
{
   char      name[32];
   uintptr_t handle;
   uint32_t  caller_address;
};

struct FreertosAssertFailureRecord
{
   char     expr[32];
   char     file[32];
   int32_t  line;
   uint32_t caller_address;
};

}   // namespace error

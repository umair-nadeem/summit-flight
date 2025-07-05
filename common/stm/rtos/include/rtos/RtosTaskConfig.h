#pragma once

#include <cstdint>

#include "FreeRTOS.h"

namespace rtos
{

using TaskFunction = void (*)(void*);
using TCB = StaticTask_t;

struct RtosTaskConfig
{
   TaskFunction func;
   const char * const name;
   const size_t stack_depth_in_words;
   void * const params;
   size_t priority;
   uint32_t* stack_buffer;
   TCB& task_block;
};

}  // namespace rtos

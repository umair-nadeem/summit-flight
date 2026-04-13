#pragma once

#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif

   void freertos_config_assert(const char* expr, const char* file, int line);

#ifdef __cplusplus
}   // extern "C"
#endif

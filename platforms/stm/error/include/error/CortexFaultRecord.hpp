#pragma once

#include "main.h"

namespace error
{

struct CortexFaultRecord
{
   uint32_t stacked_lr;    // link register
   uint32_t stacked_pc;    // program counter
   uint32_t stacked_psr;   // program status register

   uint32_t CFSR;          // Configurable Fault Status Register
   uint32_t HFSR;          // HardFault Status Register
   uint32_t MMFAR;         // MemManage Fault Address Register
   uint32_t BFAR;          // BusFault Address Register

   enum class FaultType : uint8_t
   {
      hardfault,
      busfault,
      memmanage,
      usagefault,
   };

   FaultType type;
};

}   // namespace error

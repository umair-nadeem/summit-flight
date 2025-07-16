#include "error/error_record.h"

extern "C"
{

   void common_fault_handler(uint32_t* stack_frame, error::CortexFaultRecord::FaultType type)
   {
      error::CortexFaultRecord record{};
      record.stacked_lr  = stack_frame[5];
      record.stacked_pc  = stack_frame[6];
      record.stacked_psr = stack_frame[7];

      record.CFSR  = SCB->CFSR;
      record.HFSR  = SCB->HFSR;
      record.MMFAR = SCB->MMFAR;
      record.BFAR  = SCB->BFAR;

      record.type = type;

      add_error_record(record);
   }

   void BusFault_Handler(void)
   {
      __asm volatile(
          "tst lr, #4                     \n"   // Test bit 2 of LR to determine stack used
          "ite eq                         \n"   // If-Then-Else
          "mrseq r0, msp                  \n"   // If EQ: Main stack was used
          "mrsne r0, psp                  \n"   // If NE: Process stack was used
          "mov r1, %0                     \n"   // Load fault type as second argument
          "b common_fault_handler         \n"   // Branch to common handler
          :
          : "i"(static_cast<int>(error::CortexFaultRecord::FaultType::busfault))
          : "r0", "r1");
   }

   void HardFault_Handler(void)
   {
      __asm volatile(
          "tst lr, #4                     \n"
          "ite eq                         \n"
          "mrseq r0, msp                  \n"
          "mrsne r0, psp                  \n"
          "mov r1, %0                     \n"
          "b common_fault_handler         \n"
          :
          : "i"(static_cast<int>(error::CortexFaultRecord::FaultType::hardfault))
          : "r0", "r1");
   }

   void MemManage_Handler(void)
   {
      __asm volatile(
          "tst lr, #4                     \n"
          "ite eq                         \n"
          "mrseq r0, msp                  \n"
          "mrsne r0, psp                  \n"
          "mov r1, %0                     \n"
          "b common_fault_handler         \n"
          :
          : "i"(static_cast<int>(error::CortexFaultRecord::FaultType::memmanage))
          : "r0", "r1");
   }

   void UsageFault_Handler(void)
   {
      __asm volatile(
          "tst lr, #4                     \n"
          "ite eq                         \n"
          "mrseq r0, msp                  \n"
          "mrsne r0, psp                  \n"
          "mov r1, %0                     \n"
          "b common_fault_handler         \n"
          :
          : "i"(static_cast<int>(error::CortexFaultRecord::FaultType::usagefault))
          : "r0", "r1");
   }

   void NMI_Handler(void)
   {
      while (true)
      {
         __asm("BKPT #0");
      }
   }

}   // extern "C"

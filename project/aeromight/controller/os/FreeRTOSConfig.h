// clang-format off

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "system_stm32f4xx.h"
#include "error/freertos_hooks.h"

/******************************************************************************/
/* Hardware and scheduling definitions. ***************************************/
/******************************************************************************/

#define configCPU_CLOCK_HZ    ( ( unsigned long ) SystemCoreClock )
#define configTICK_RATE_HZ                         1000
#define configUSE_PREEMPTION                       1
#define configUSE_TIME_SLICING                     1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION    0
#define configUSE_TICKLESS_IDLE                    0
#define configMAX_PRIORITIES                       5
#define configMINIMAL_STACK_SIZE                   128
#define configMAX_TASK_NAME_LEN                    16
#define configTICK_TYPE_WIDTH_IN_BITS              TICK_TYPE_WIDTH_32_BITS
#define configIDLE_SHOULD_YIELD                    1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES      1
#define configQUEUE_REGISTRY_SIZE                  0
#define configENABLE_BACKWARD_COMPATIBILITY        0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS    0
#define configUSE_MINI_LIST_ITEM                   1
#define configSTACK_DEPTH_TYPE                     size_t
#define configMESSAGE_BUFFER_LENGTH_TYPE           size_t
#define configHEAP_CLEAR_MEMORY_ON_FREE            1
#define configSTATS_BUFFER_MAX_LENGTH              0xFFFF
#define configUSE_NEWLIB_REENTRANT                 0

/******************************************************************************/
/* Software timer related definitions. ****************************************/
/******************************************************************************/

#define configUSE_TIMERS                1
#define configTIMER_TASK_PRIORITY       ( configMAX_PRIORITIES - 1 )
#define configTIMER_TASK_STACK_DEPTH    configMINIMAL_STACK_SIZE
#define configTIMER_QUEUE_LENGTH        10

/******************************************************************************/
/* Event Group related definitions. *******************************************/
/******************************************************************************/

#define configUSE_EVENT_GROUPS      1
#define configUSE_STREAM_BUFFERS    1

/******************************************************************************/
/* Memory allocation related definitions. *************************************/
/******************************************************************************/

#define configSUPPORT_STATIC_ALLOCATION              1
#define configSUPPORT_DYNAMIC_ALLOCATION             0
#define configTOTAL_HEAP_SIZE                        0
#define configAPPLICATION_ALLOCATED_HEAP             0
#define configSTACK_ALLOCATION_FROM_SEPARATE_HEAP    0
#define configENABLE_HEAP_PROTECTOR                  0
 
/******************************************************************************/
/* Interrupt nesting behaviour configuration. *********************************/
/******************************************************************************/

#define configKERNEL_INTERRUPT_PRIORITY              (15 << 4)
#define configMAX_SYSCALL_INTERRUPT_PRIORITY         (5 << 4)
#define configMAX_API_CALL_INTERRUPT_PRIORITY        (5 << 4)

/******************************************************************************/
/* Hook and callback function related definitions. ****************************/
/******************************************************************************/

#define configUSE_IDLE_HOOK                   0
#define configUSE_TICK_HOOK                   0
#define configUSE_MALLOC_FAILED_HOOK          1
#define configUSE_DAEMON_TASK_STARTUP_HOOK    0
#define configUSE_SB_COMPLETED_CALLBACK       0
#define configCHECK_FOR_STACK_OVERFLOW        2

/******************************************************************************/
/* Run time and task stats gathering related definitions. *********************/
/******************************************************************************/

#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/******************************************************************************/
/* Co-routine related definitions. ********************************************/
/******************************************************************************/

#define configUSE_CO_ROUTINES              0
#define configMAX_CO_ROUTINE_PRIORITIES    1

/******************************************************************************/
/* Debugging assistance. ******************************************************/
/******************************************************************************/

/* *INDENT-OFF* */
#define configASSERT( x )    if( ( x ) == 0 ) { freertos_config_assert(#x, __FILE__, __LINE__); }
/* *INDENT-ON* */

/******************************************************************************/
/* FreeRTOS MPU specific definitions. *****************************************/
/******************************************************************************/

#define configINCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS    0
#define configTOTAL_MPU_REGIONS                                   8
#define configTEX_S_C_B_FLASH                                     0x07UL
#define configTEX_S_C_B_SRAM                                      0x07UL
#define configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY               1
#define configALLOW_UNPRIVILEGED_CRITICAL_SECTIONS                0
#define configUSE_MPU_WRAPPERS_V1                                 0
#define configPROTECTED_KERNEL_OBJECT_POOL_SIZE                   0
#define configSYSTEM_CALL_STACK_SIZE                              0
#define configENABLE_ACCESS_CONTROL_LIST                          0

/******************************************************************************/
/* SMP( Symmetric MultiProcessing ) Specific Configuration definitions. *******/
/******************************************************************************/

#define configNUMBER_OF_CORES                     1
#define configRUN_MULTIPLE_PRIORITIES             0
#define configUSE_CORE_AFFINITY                   0
#define configTASK_DEFAULT_CORE_AFFINITY          tskNO_AFFINITY
#define configUSE_TASK_PREEMPTION_DISABLE         0
#define configUSE_PASSIVE_IDLE_HOOK               0
#define configTIMER_SERVICE_TASK_CORE_AFFINITY    tskNO_AFFINITY

/******************************************************************************/
/* ARMv8-M secure side port related definitions. ******************************/
/******************************************************************************/

#define secureconfigMAX_SECURE_CONTEXTS        0

/* Defines the kernel provided implementation of
 * vApplicationGetIdleTaskMemory() and vApplicationGetTimerTaskMemory()
 * to provide the memory that is used by the Idle task and Timer task
 * respectively. The application can provide it's own implementation of
 * vApplicationGetIdleTaskMemory() and vApplicationGetTimerTaskMemory() by
 * setting configKERNEL_PROVIDED_STATIC_MEMORY to 0 or leaving it undefined. */
#define configKERNEL_PROVIDED_STATIC_MEMORY    1

/******************************************************************************/
/* ARMv8-M port Specific Configuration definitions. ***************************/
/******************************************************************************/

#define configENABLE_TRUSTZONE              0
#define configRUN_FREERTOS_SECURE_ONLY      0
#define configENABLE_MPU                    0
#define configENABLE_FPU                    1
#define configENABLE_MVE                    0

#define configCHECK_HANDLER_INSTALLATION    1

/******************************************************************************/
/* Definitions that include or exclude functionality. *************************/
/******************************************************************************/

#define configUSE_TASK_NOTIFICATIONS           1
#define configUSE_MUTEXES                      1
#define configUSE_RECURSIVE_MUTEXES            1
#define configUSE_COUNTING_SEMAPHORES          1
#define configUSE_QUEUE_SETS                   0
#define configUSE_APPLICATION_TASK_TAG         0
#define configUSE_POSIX_ERRNO                  0

#define INCLUDE_vTaskPrioritySet               1
#define INCLUDE_uxTaskPriorityGet              1
#define INCLUDE_vTaskDelete                    1
#define INCLUDE_vTaskSuspend                   0
#define INCLUDE_vTaskDelayUntil                1
#define INCLUDE_vTaskDelay                     0
#define INCLUDE_xTaskGetSchedulerState         0
#define INCLUDE_xTaskGetCurrentTaskHandle      1
#define INCLUDE_uxTaskGetStackHighWaterMark    1
#define INCLUDE_xTaskGetIdleTaskHandle         0
#define INCLUDE_eTaskGetState                  0
#define INCLUDE_xTimerPendFunctionCall         0
#define INCLUDE_xTaskAbortDelay                0
#define INCLUDE_xTaskGetHandle                 0
#define INCLUDE_xTaskResumeFromISR             0

#endif /* FREERTOS_CONFIG_H */

// clang-format on

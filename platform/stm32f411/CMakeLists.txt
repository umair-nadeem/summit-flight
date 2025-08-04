only_for_toolchain(stm_m4)

add_library(stm32f411 INTERFACE)

file(GLOB_RECURSE sources_files ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/*.c)

list(APPEND sources_files
   ${CMAKE_CURRENT_LIST_DIR}/Core/Src/main.c
   ${CMAKE_CURRENT_LIST_DIR}/Core/Src/stm32f4xx_hal_msp.c
   ${CMAKE_CURRENT_LIST_DIR}/Core/Src/syscalls.c
   ${CMAKE_CURRENT_LIST_DIR}/Core/Src/system_stm32f4xx.c
   ${CMAKE_CURRENT_LIST_DIR}/startup_stm32f411xe.s
)

target_sources(stm32f411 INTERFACE ${sources_files})

set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/startup_stm32f411xe.s
   PROPERTIES LANGUAGE ASM
)

target_include_directories(stm32f411 SYSTEM INTERFACE
   ${CMAKE_CURRENT_LIST_DIR}/Core/Inc
   ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
   ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Include
   ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
)

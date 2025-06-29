only_for_toolchain(stm)

file(GLOB_RECURSE stm32f411_sources
    ${CMAKE_CURRENT_LIST_DIR}/Core/Src/*.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/*.c
)

list(APPEND stm32f411_sources
    ${CMAKE_CURRENT_LIST_DIR}/startup_stm32f411xe.s
)

add_library(stm32f411_platform OBJECT ${stm32f411_sources})

set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/startup_stm32f411xe.s
    PROPERTIES LANGUAGE ASM
)

target_include_directories(stm32f411_platform SYSTEM PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/Core/Inc
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
)

target_compile_options(stm32f411_platform PUBLIC
    # Preprocessor Macros
    -DSTM32F411xE
    -DARM_MATH_CM4
    -DARM_MATH_MATRIX_CHECK
    -DARM_MATH_ROUNDING
    -DUSE_HAL_DRIVER
    -DUSE_FULL_LL_DRIVER

    # Architecture-Specific
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
    -mcpu=cortex-m4
    -mthumb
    -mthumb-interwork
    -mno-unaligned-access

    # Linker-Placement/Memory-Optimization Flags
    -ffunction-sections
    -fdata-sections
    -fno-common

    # C++ Flags and Warnings
    $<$<COMPILE_LANGUAGE:CXX>:
    -fno-rtti
    -fno-exceptions
    -Weffc++
    -Wabi-tag
    >

    ## Optimization & Performance
    -O2
    -g

    # explicit -O3 flags
    -fgcse-after-reload
    -floop-interchange
    -fpeel-loops
    -fpredictive-commoning
    -fsplit-loops
    -fsplit-paths
    -ftree-loop-distribution
    -ftree-partial-pre
    -fvect-cost-model=dynamic
)

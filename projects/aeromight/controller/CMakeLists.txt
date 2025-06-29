only_for_toolchain(stm)

add_library(aeromight_lib INTERFACE)

target_sources(aeromight_lib INTERFACE 
    aeromight.cpp
)

target_link_libraries(aeromight_lib INTERFACE
    stm32f411
)

target_compile_options(aeromight_lib INTERFACE
    # Preprocessor Macros
    -DSTM32F411xE
    -DARM_MATH_CM4
    -DARM_MATH_MATRIX_CHECK
    -DARM_MATH_ROUNDING
    -DUSE_HAL_DRIVER
    -DUSE_FULL_LL_DRIVER
)

add_toolchain_flags(aeromight_lib)

add_cortex_m4_compile_flags(aeromight_lib)

add_cortex_m4_link_flags(aeromight_lib ${CMAKE_CURRENT_SOURCE_DIR}/STM32F411XX_FLASH.ld)

add_executable(aeromight)

target_link_libraries(aeromight aeromight_lib)

export_executable(aeromight ${CMAKE_BINARY_DIR}/executables)

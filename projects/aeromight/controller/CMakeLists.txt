only_for_toolchain(stm)

add_library(aeromight_lib INTERFACE)

target_sources(aeromight_lib INTERFACE 
    main.cpp
    $<TARGET_OBJECTS:stm32f411_platform>
)

target_compile_options(aeromight_lib INTERFACE
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
)

target_link_options(aeromight_lib INTERFACE
    -Wl,-gc-sections
    -Wl,--unresolved-symbols=report-all
    -mcpu=cortex-m4
    -mthumb
    -mthumb-interwork
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16  
    -specs=nano.specs -specs=nosys.specs
    -T ${CMAKE_CURRENT_SOURCE_DIR}/STM32F411XX_FLASH.ld
)

add_executable(aeromight)

target_link_libraries(aeromight aeromight_lib)


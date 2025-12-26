only_for_toolchain(stm_m4)

add_library(aeromight_lib INTERFACE)

target_include_directories(aeromight_lib INTERFACE 
   ${CMAKE_CURRENT_LIST_DIR}/os
   ${CMAKE_CURRENT_LIST_DIR}/platform
)

target_sources(aeromight_lib INTERFACE 
   ${CMAKE_CURRENT_LIST_DIR}/platform/main.c
   ${CMAKE_CURRENT_LIST_DIR}/os/port.c
   ${CMAKE_CURRENT_LIST_DIR}/sys_clock.cpp

   aeromight.cpp
   logging_task.cpp
   imu_task.cpp
   control_task.cpp
   barometer_task.cpp
   flight_manager_task.cpp
   health_monitoring_task.cpp
   radio_link_task.cpp
)

target_link_libraries(aeromight_lib INTERFACE
   crsf
   hw
   rtos
   sml
   stm32f411

   aeromight_boundaries
   aeromight_barometer
   aeromight_control
   aeromight_estimation
   aeromight_flight
   aeromight_health
   aeromight_imu
   aeromight_link
   bmp390
   error
   interfaces
   mpu6500
   logging
)

target_compile_options(aeromight_lib INTERFACE
   # Preprocessor Macros
   -DSTM32F411xE
   -DARM_MATH_CM4
   -DARM_MATH_MATRIX_CHECK
   -DARM_MATH_ROUNDING
   -DUSE_FULL_LL_DRIVER
   -DUSE_FULL_ASSERT
)

add_compile_flags(aeromight_lib)

add_cortex_m4_link_flags(aeromight_lib ${CMAKE_CURRENT_LIST_DIR}/STM32F411XX_FLASH.ld)

add_executable(aeromight)

target_link_libraries(aeromight aeromight_lib)

export_executable(aeromight ${CMAKE_BINARY_DIR}/artifacts)

print_size(
   aeromight 
   ${CMAKE_BINARY_DIR}/artifacts/aeromight
   ${CMAKE_SOURCE_DIR}/cmake/print_size_info.cmake
)

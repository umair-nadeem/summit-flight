only_for_toolchain(native)

add_library(aeromight_sitl_lib INTERFACE)

target_sources(aeromight_sitl_lib INTERFACE 
   aeromight_sitl.cpp
)

target_link_libraries(aeromight_sitl_lib INTERFACE

   aeromight_boundaries
   aeromight_control
   aeromight_estimation
   aeromight_system
   aeromight_health
   aeromight_rc
   barometer
   control
   crsf
   dshot
   event_handling
   error
   imu
   interfaces
   led
   logging
   power
   rc
)

add_compile_flags(aeromight_sitl_lib)

# SITL-specific compile flags (only for this target)
target_compile_options(aeromight_sitl_lib INTERFACE
    $<$<COMPILE_LANGUAGE:CXX>:
        -pthread
    >
)

target_link_options(aeromight_sitl_lib INTERFACE
    -pthread
)

add_executable(aeromight_sitl)

target_link_libraries(aeromight_sitl aeromight_sitl_lib)

export_executable(aeromight_sitl ${CMAKE_BINARY_DIR}/artifacts)

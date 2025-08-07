if (TOOLCHAIN STREQUAL "native")

   # google test
   add_subdirectory(googletest)

elseif (TOOLCHAIN STREQUAL "stm_m4")

   # stm hal
   add_subdirectory(stm32f411)

   # freertos
   add_library(freertos INTERFACE)
   set(sources_files
      freertos/event_groups.c
      freertos/list.c
      freertos/queue.c
      freertos/stream_buffer.c
      freertos/tasks.c
      freertos/timers.c
   )
   target_sources(freertos INTERFACE ${sources_files})
   target_include_directories(freertos SYSTEM INTERFACE freertos/include)

endif()

# printf
add_library(printf INTERFACE)
set_property(TARGET printf PROPERTY C_STANDARD 99)
set_property(TARGET printf PROPERTY C_STANDARD_REQUIRED ON)
set_property(TARGET printf PROPERTY C_EXTENSIONS OFF)
target_sources(printf INTERFACE ${CMAKE_CURRENT_LIST_DIR}/printf/src/printf/printf.c)
target_include_directories(printf SYSTEM INTERFACE printf/src)

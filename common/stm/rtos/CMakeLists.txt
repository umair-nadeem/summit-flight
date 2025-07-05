set(OS_DIR "${CMAKE_SOURCE_DIR}/external/freertos")

add_library(freertos INTERFACE)

set(sources_files
    ${OS_DIR}/event_groups.c
    ${OS_DIR}/list.c
    ${OS_DIR}/queue.c
    ${OS_DIR}/stream_buffer.c
    ${OS_DIR}/tasks.c
    ${OS_DIR}/timers.c
)

target_sources(freertos INTERFACE ${sources_files})

target_include_directories(freertos SYSTEM INTERFACE ${OS_DIR}/include)


############################################################################################


add_library(rtos INTERFACE)

target_sources(rtos INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/hooks.cpp)

target_link_libraries(rtos INTERFACE freertos)

target_include_directories(rtos INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

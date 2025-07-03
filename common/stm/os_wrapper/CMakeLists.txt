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

target_include_directories(freertos SYSTEM INTERFACE
    ${OS_DIR}/include)


##################################################################


add_library(os_wrapper INTERFACE)

target_link_libraries(os_wrapper INTERFACE freertos)

target_include_directories(os_wrapper INTERFACE ${CMAKE_CURRENT_LIST_DIR})

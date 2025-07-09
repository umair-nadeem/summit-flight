add_library(aeromight_sensors INTERFACE)

target_include_directories(aeromight_sensors INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

if (TOOLCHAIN STREQUAL "native")
   add_subdirectory(test)
endif()

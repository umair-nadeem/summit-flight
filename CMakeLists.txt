cmake_minimum_required(VERSION 3.28)
project(summitflight)
enable_language(C CXX ASM)

message("Toolchain file ${CMAKE_TOOLCHAIN_FILE} is selected")

# cmake utitilies
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

include(only_for_toolchain)
include(cortex_m4_flags)

add_subdirectory(platform)
add_subdirectory(projects)

cmake_minimum_required(VERSION 3.28)
project(summitflight)
enable_language(C CXX ASM)

enable_testing()

message("Toolchain file ${CMAKE_TOOLCHAIN_FILE} is selected")

# cmake utitilies
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

include(compile_flags)
include(export_executable)
include(only_for_toolchain)
include(add_module)
include(add_module_test)

add_subdirectory(common)
add_subdirectory(external)
add_subdirectory(platform)
add_subdirectory(project)

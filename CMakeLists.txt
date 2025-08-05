cmake_minimum_required(VERSION 3.28)
project(summitflight)
enable_language(C CXX ASM)

enable_testing()

message("Toolchain file ${CMAKE_TOOLCHAIN_FILE} is selected")

# cmake utitilies
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

include(compile_flags)
include(postbuild_operations)
include(only_for_toolchain)
include(add_module)
include(add_module_test)
include(print_size_info)

add_subdirectory(common)
add_subdirectory(external)
add_subdirectory(project)

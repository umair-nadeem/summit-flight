function(add_module)
    cmake_parse_arguments(args "" "" "SOURCES;LIBS" ${ARGN})

    # get module name
    get_filename_component(module_name "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

    add_library(${module_name} INTERFACE)

    target_include_directories(${module_name} INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
   
    if (args_SOURCES)
        target_sources(${module_name} INTERFACE ${args_SOURCES})
    endif()
       
    if(args_LIBS)
        target_link_libraries(${module_name} INTERFACE ${args_LIBS})
    endif()

    if (TOOLCHAIN STREQUAL "native")
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
            add_subdirectory(test)
        endif()
    endif()

endfunction()

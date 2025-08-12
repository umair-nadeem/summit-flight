include(common_flags)
include(cortex_m4_flags)
include(native_flags)

function(add_compile_flags library)

    set(compile_flags_lib "${library}_compile_flags")  
    add_library(${compile_flags_lib} INTERFACE)

    # add universal flags
    add_common_flags(${compile_flags_lib})

    # apply toolchain specific flags
    if (TOOLCHAIN STREQUAL "stm_m4")
        add_cortex_m4_compile_flags(${compile_flags_lib})
        message(STATUS "Adding cortex m4 flags")
    elseif(TOOLCHAIN STREQUAL "native")
        add_native_flags(${compile_flags_lib})
        message(STATUS "Adding native toolchain flags")
    endif()

    get_target_property(lib_type ${library} TYPE)

    if(lib_type STREQUAL "INTERFACE_LIBRARY")
        target_link_libraries(${library} INTERFACE ${compile_flags_lib})
    else()
        target_link_libraries(${library} ${compile_flags_lib})
    endif()
    
endfunction()

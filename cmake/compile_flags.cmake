include(common_flags)
include(cortex_m4_flags)
include(native_flags)

function(add_compile_flags library)
    # add universal flags
    add_common_flags(${library})

    # apply toolchain specific flags
    if (TOOLCHAIN STREQUAL "stm_m4")
        add_cortex_m4_compile_flags(${library})
        message(STATUS "Adding cortex m4 flags")
    elseif(TOOLCHAIN STREQUAL "native")
        add_native_flags(${library})
        message(STATUS "Adding native toolchain flags")
    endif()

endfunction()

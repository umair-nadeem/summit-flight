function(add_native_flags library)
    target_compile_options(${library} INTERFACE
        -g3
        -O0
    )
endfunction()

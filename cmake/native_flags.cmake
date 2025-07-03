function(add_native_flags library)
    target_compile_options(${library} INTERFACE
        -fsanitize=address
        -g3
    )
endfunction()

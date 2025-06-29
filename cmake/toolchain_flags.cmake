function(add_toolchain_flags interface_library)
    target_compile_options(${interface_library} INTERFACE

        $<$<COMPILE_LANGUAGE:C>:-std=gnu17>

        $<$<COMPILE_LANGUAGE:CXX>:-std=c++20

        -fmessage-length=0
        -fno-threadsafe-statics
        -fnothrow-opt
        -funsigned-bitfields
        -funsigned-char

        -Wall
        -Wcast-qual
        -Wcatch-value=3
        -Wconditionally-supported
        -Wconversion
        -Wdangling-else
        -Wdate-time
        -Wdeprecated-copy-dtor
        -Wduplicated-branches
        -Wduplicated-cond
        -Werror
        -Wextra
        -Wextra-semi
        -Wfloat-conversion
        -Wfloat-equal
        -Wformat=2
        -Wlogical-op
        -Wmismatched-tags
        -Wmissing-include-dirs
        -Wnoexcept
        -Wnon-virtual-dtor
        -Wnull-dereference
        -Wold-style-cast
        -Woverloaded-virtual
        -Wpedantic
        -Wredundant-decls
        -Wredundant-tags
        -Wshadow
        -Wsign-conversion
        -Wsign-promo
        -Wstrict-null-sentinel
        $<$<STREQUAL:${TOOLCHAIN},stm>:-Wstrict-overflow=5>
        -Wsuggest-final-methods
        -Wsuggest-final-types
        -Wsuggest-override
        -Wswitch-default
        -Wswitch-enum
        -Wtrampolines
        -Wundef
        -Wuninitialized
        -Wzero-as-null-pointer-constant
        -Wzero-length-bounds
        >
    )
endfunction()
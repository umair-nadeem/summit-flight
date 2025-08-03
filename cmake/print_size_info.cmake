# If this file is executed directly in POST_BUILD mode
if(EXEC_FILE AND EXISTS "${EXEC_FILE}")
    # Compute Flash and RAM usage
    execute_process(
        COMMAND ${CMAKE_SIZE} --format=SysV ${EXEC_FILE}
        OUTPUT_VARIABLE SIZE_OUTPUT
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    # Extract section + size
    string(REGEX MATCHALL "(\\.[A-Za-z0-9_]+)[ \t]+([0-9]+)" SECTIONS "${SIZE_OUTPUT}")

    set(FLASH 0)
    set(RAM 0)

    foreach(ENTRY IN LISTS SECTIONS)
        string(REGEX MATCH "(\\.[A-Za-z0-9_]+)[ \t]+([0-9]+)" _ "${ENTRY}")
        set(SECTION "${CMAKE_MATCH_1}")
        set(SIZE "${CMAKE_MATCH_2}")

        # Flash = .text + .rodata + .data
        if(SECTION STREQUAL ".text" OR SECTION STREQUAL ".data" OR SECTION STREQUAL ".rodata")
            math(EXPR FLASH "${FLASH} + ${SIZE}")
        endif()

        # RAM = .data + .bss + ._user_heap_stack
        if(SECTION STREQUAL ".data" OR SECTION STREQUAL ".bss" OR SECTION STREQUAL "._user_heap_stack")
            math(EXPR RAM "${RAM} + ${SIZE}")
        endif()
    endforeach()

    # Convert to KB and print
    math(EXPR FLASH_K "${FLASH} / 1024")
    math(EXPR RAM_K "${RAM} / 1024")

    message(STATUS "FLASH: ${FLASH_K}K, RAM: ${RAM_K}K")
endif()

function(export_executable exe_target export_dir)
    add_custom_command(
        TARGET ${exe_target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${export_dir}
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${exe_target}> ${export_dir}
        COMMENT "Exporting ${exe_target} to ${export_dir}"
    )
endfunction()


# print ram and flash size
function(print_size exe_target exe_file script_file)
    add_custom_command(
        TARGET ${exe_target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -DEXEC_FILE=${exe_file} -DCMAKE_SIZE=${CMAKE_SIZE} -P ${script_file}
        COMMENT "Calculating Flash and RAM usage for ${exe_target}"
    )
endfunction()

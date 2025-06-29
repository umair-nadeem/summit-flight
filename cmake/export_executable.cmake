function(export_executable exe_target export_dir)
    add_custom_command(
        TARGET ${exe_target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${export_dir}
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${exe_target}> ${export_dir}
        COMMENT "Exporting ${exe_target} to ${export_dir}"
    )
endfunction()

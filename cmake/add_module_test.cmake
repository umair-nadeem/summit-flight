only_for_toolchain(native)

function(add_module_test)
   cmake_parse_arguments(args "" "" "SOURCES;LIBS" ${ARGN})

   # get module name
   get_filename_component(module_dir "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
   get_filename_component(module_name "${module_dir}" NAME)
   set(test_target "${module_name}_test")

   add_executable(${test_target} ${args_SOURCES})
   target_link_libraries(${test_target}
      ${module_name}
      gtest
      gtest_main
      gmock_main
      ${args_LIBS}
   )

   add_compile_flags(${test_target})

   set_target_properties(${test_target} PROPERTIES EXCLUDE_FROM_ALL FALSE)

   # register with ctest
   add_test(NAME ${test_target}
         COMMAND ${test_target})

endfunction()

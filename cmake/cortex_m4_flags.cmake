function(add_cortex_m4_compile_flags library)
    target_compile_options(${library} INTERFACE
        # Architecture-Specific
       -mfloat-abi=hard
       -mfpu=fpv4-sp-d16
       -mcpu=cortex-m4
       -mthumb
       -mthumb-interwork
       -mno-unaligned-access

       # Linker-Placement/Memory-Optimization Flags
       -ffunction-sections
       -fdata-sections
       -fno-common

       # C++ Flags and Warnings
       $<$<COMPILE_LANGUAGE:CXX>:
       -fno-exceptions
       -fno-rtti
       -Wabi-tag
       -Weffc++
       -Wstrict-overflow=5
       >

       ## Optimization & Performance
       -O2
       -g

       # explicit -O3 flags
       -fgcse-after-reload
       -floop-interchange
       -fpeel-loops
       -fpredictive-commoning
       -fsplit-loops
       -fsplit-paths
       -ftree-loop-distribution
       -ftree-partial-pre
       -fvect-cost-model=dynamic
   )
endfunction()

function(add_cortex_m4_link_flags library linker_script)
    target_link_options(${library} INTERFACE
        -Wl,-gc-sections
        -Wl,--unresolved-symbols=report-all
        -mcpu=cortex-m4
        -mthumb
        -mthumb-interwork
        -mfloat-abi=hard
        -mfpu=fpv4-sp-d16  
        -specs=nano.specs -specs=nosys.specs
        -T ${linker_script}
    )
endfunction()

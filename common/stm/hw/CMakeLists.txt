add_library(hw INTERFACE)

target_include_directories(hw INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

target_sources(hw INTERFACE
   src/dma/dma.cpp
   src/uart/Transmitter.cpp
   src/uart/uart.cpp
)

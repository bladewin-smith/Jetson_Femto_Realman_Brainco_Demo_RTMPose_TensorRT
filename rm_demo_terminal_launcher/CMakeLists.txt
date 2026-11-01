cmake_minimum_required(VERSION 3.10)
project(rm_demo_terminal_launcher LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(rm_brainco_orbbec_demo_terminal
    src/main.cpp
)

install(TARGETS rm_brainco_orbbec_demo_terminal
    RUNTIME DESTINATION bin
)

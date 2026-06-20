cmake_minimum_required(VERSION 3.16)

project(PayloadControl C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_library(payload_control
    src/payload_control.c
    src/payload_mavlink_bridge.c
)

target_include_directories(payload_control PUBLIC include)

add_executable(payload_control_test
    tests/test_payload_control.c
)

target_link_libraries(payload_control_test PRIVATE payload_control)

enable_testing()
add_test(NAME payload_control_test COMMAND payload_control_test)

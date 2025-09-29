cmake_minimum_required(VERSION 3.15)

project(uav_height_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions("-Wall -Wextra -Werror")

find_package(MAVSDK REQUIRED)

add_executable(uav_height_control
    main.cpp
    flight_operator.cpp
    height_control_loop.cpp
    regulator.cpp
)

target_link_libraries(uav_height_control
    MAVSDK::mavsdk
)
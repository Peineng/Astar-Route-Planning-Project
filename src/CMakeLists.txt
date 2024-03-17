cmake_minimum_required(VERSION 3.27)
project(src)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(src
        main.cpp
        model.cpp
        model.h
        render.cpp
        render.h
        route_model.cpp
        route_model.h
        route_planner.cpp
        route_planner.h)

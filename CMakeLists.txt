cmake_minimum_required(VERSION 3.13)
project(TSP_Implementation)

set(CMAKE_CXX_STANDARD 17)

include_directories(
        include
)

set(SOURCE_FILES
        src/tsp_setup.cpp
        src/TSP-template.cpp
        )

add_executable(tsp ${SOURCE_FILES} main.cpp)
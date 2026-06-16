cmake_minimum_required(VERSION 3.14)
project(pointcloud_baseline LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

file(GLOB BASELINE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_executable(baseline main.cpp ${BASELINE_SOURCES})

target_include_directories(baseline PRIVATE include)

find_package(Threads REQUIRED)
target_link_libraries(baseline PRIVATE Threads::Threads)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(baseline PRIVATE -Wall -Wextra -Wpedantic -O3)
endif()

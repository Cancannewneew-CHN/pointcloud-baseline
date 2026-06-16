cmake_minimum_required(VERSION 3.14)
project(pointcloud_baseline LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(baseline
    main.cpp
    src/ply_io.cpp
    src/roi_crop.cpp
    src/voxel_downsample.cpp
    src/statistical_filter.cpp
    src/diff_segmentation.cpp
    src/pipeline.cpp
)

target_include_directories(baseline PRIVATE include)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(baseline PRIVATE -Wall -Wextra -Wpedantic)
endif()

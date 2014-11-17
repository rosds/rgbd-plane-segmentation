cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(voxel_grid)

find_package(PCL 1.2 REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SRC src/main.cpp src/Metric.cpp src/GHDistance.cpp)

add_executable (gh ${SRC})
add_executable (voxel_grid src/voxel_grid.cpp)

target_link_libraries (gh ${PCL_LIBRARIES})
target_link_libraries (voxel_grid ${PCL_LIBRARIES})

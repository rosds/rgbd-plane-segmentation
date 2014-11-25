cmake_minimum_required(VERSION 2.8)

project(plane_detection)
set(CMAKE_BUILD_TYPE Debug)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

set(OpenCV_DIR "/opt/ros/hydro/share/OpenCV")
#set(OpenCV_DIR "/opt/ros/fuerte/share/OpenCV")
#set(PCL_DIR "/opt/ros/fuerte/share/pcl-1.5")

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)
find_package(CUDA)
find_package(Doxygen)

if(OPENMP_FOUND)
    set (WITH_OMP ON)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif(OPENMP_FOUND)

if(CUDA_FOUND)
    set (WITH_CUDA ON)
    if(CMAKE_BUILD_TYPE MATCHES DEBUG)
        set (CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -g -G)
    endif(CMAKE_BUILD_TYPE MATCHES DEBUG) 
    cuda_include_directories(include)
    cuda_compile(CUDA_FUNC src/cuda/Frame.cu)
    cuda_add_library(cuacceleration SHARED 
        src/Frame.cpp 
        src/cuda/Frame.cu
        src/utils.cpp
    )
    add_definitions(-DWITH_CUDA)
endif(CUDA_FOUND)

if(DOXYGEN_FOUND)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
        ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
        )
endif(DOXYGEN_FOUND)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

add_definitions("-Wall -g")
add_definitions(${PCL_DEFINITIONS})

add_executable(plane_segmentation 
    src/main.cpp 
    src/utils.cpp
    src/DataSet.cpp 
    src/Frame.cpp
    src/Transform3D.cpp
    src/CropPlaneInliers.cpp
)

target_link_libraries(plane_segmentation ${PCL_LIBRARIES} ${PCL_LIBS}
    ${OpenCV_LIBS} cuacceleration)

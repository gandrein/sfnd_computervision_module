cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")

project(camera_lidar_fusion_ttc)

find_package(OpenCV 4.1 REQUIRED)

set(DEPS "${CMAKE_CURRENT_SOURCE_DIR}/deps")

link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Executable for create matrix exercise
add_executable(3D_object_tracking
            src/main.cpp
            src/cameraFusion.cpp
            src/lidarData.cpp
            src/matchingFeatures2D.cpp
            src/objectDetection2D.cpp
            src/utils.cpp)
target_link_libraries(3D_object_tracking ${OpenCV_LIBRARIES})

target_include_directories(3D_object_tracking PRIVATE 
            ${OpenCV_INCLUDE_DIRS}
            ${DEPS}/tclap-1.2.2)

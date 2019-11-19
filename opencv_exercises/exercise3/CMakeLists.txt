cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(camera_fusion_ex3)

find_package(OpenCV 4.1 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Executables for load image exercises
add_executable (load_image_1 load_image_1.cpp)
target_link_libraries(load_image_1 ${OpenCV_LIBRARIES})

add_executable (load_image_2 load_image_2.cpp)
target_link_libraries(load_image_2 ${OpenCV_LIBRARIES})

add_executable (load_image_3 load_image_3.cpp)
target_link_libraries(load_image_3 ${OpenCV_LIBRARIES})

cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")

project(feature_tracking)

set(DEPS "${CMAKE_CURRENT_SOURCE_DIR}/deps")

find_package(OpenCV 4.1 REQUIRED)

# include_directories()
# link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Executable for create matrix exercise
add_executable (2D_feature_tracking src/matching2D.cpp src/main.cpp src/utils.cpp)
target_link_libraries (2D_feature_tracking ${OpenCV_LIBRARIES})
target_include_directories(2D_feature_tracking PRIVATE ${OpenCV_INCLUDE_DIRS} ${DEPS}/tclap-1.2.2)

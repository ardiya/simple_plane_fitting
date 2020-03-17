cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(simple_plane_fitting)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include/)

add_executable(simple_plane_fitting main.cpp)
target_link_libraries(simple_plane_fitting ${PCL_LIBRARIES})

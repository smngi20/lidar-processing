cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(src)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (src src.cpp)
target_link_libraries (src ${PCL_LIBRARIES})
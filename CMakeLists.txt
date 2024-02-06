cmake_minimum_required(VERSION 3.15)

project(playback)

add_definitions(-std=c++14)

add_compile_options(-Wall -mavx)

find_package(PCL 1.2 REQUIRED)
message(STATUS "Found PCL ${PCL_VERSION}")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (environment environment.cpp render/render.cpp processPointClouds.cpp)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(view_box)
# target_compile_features(${view_point_cloud} PRIVATE cxx_std_17)
set(CMAKE_CXX_STANDARD 17)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (view_box src/main.cpp)
target_link_libraries (view_box ${PCL_LIBRARIES})

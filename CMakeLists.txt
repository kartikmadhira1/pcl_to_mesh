cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(greedy_projection)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (greedy_projection src/greedy_projection.cpp)
add_executable (surf_reconstruct src/surf_reconstruct.cpp)
add_executable (reconstruct src/reconstruct.cpp)

target_link_libraries (greedy_projection ${PCL_LIBRARIES})
target_link_libraries (surf_reconstruct ${PCL_LIBRARIES})
target_link_libraries (reconstruct ${PCL_LIBRARIES})
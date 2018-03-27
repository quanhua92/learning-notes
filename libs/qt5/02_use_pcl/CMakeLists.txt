cmake_minimum_required (VERSION 3.0 FATAL_ERROR)

project(02_use_pcl)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} "02_use_pcl.cpp")
target_link_libraries (${PROJECT_NAME} ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

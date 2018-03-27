# 1. common
# ---------
cmake_minimum_required (VERSION 3.0 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# project name
project (scanner_engine)

# The version number.
set (${PROJECT_NAME}_VERSION_MAJOR 0)
set (${PROJECT_NAME}_VERSION_MINOR 1)

# 2. our lib
# ----------

# engine files
file(GLOB engine_HEADERS
  "*.h"
)
file(GLOB engine_SOURCES
  "*.cpp"
)

# add to library
add_library(${PROJECT_NAME} SHARED
${engine_SOURCES}
)

add_library(BodiData::scanner_engine ALIAS ${PROJECT_NAME})

# 3. dependencies
# ---------------

# eigen
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# pcl
find_package (PCL 1.8.1 REQUIRED NO_MODULE)

# check if pcl was found
if (PCL_FOUND)
  message ("pcl found")
else()
  message (FATAL_ERROR "cannot find pcl")
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# opencv
find_package (OpenCV REQUIRED)

# ceres
find_package(Ceres REQUIRED)

# realsense2
find_package(realsense2 REQUIRED)

# 4. links
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})

# 5. outputs
# ----------
install(TARGETS ${PROJECT_NAME}
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin)
install(FILES
${engine_HEADERS}
DESTINATION include)

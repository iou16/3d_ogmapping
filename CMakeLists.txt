cmake_minimum_required(VERSION 2.8.3)
project(edogmapping)

set(CMAKE_CXX_FLAGS "-march=native -O2 -std=c++11 -fopenmp")
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS   "-march=native -O2 -fopenmp")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "-march=native -O2 -fopenmp")
set(CMAKE_EXE_LINKER_FLAGS "-march=native -O2 -fopenmp")

find_package(catkin REQUIRED COMPONENTS
  message_filters
  roscpp
  sensor_msgs
  geometry_msgs
  tf
  pcl_ros
  pcl_conversions
  pcl_msgs
)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS 
    message_filters 
    roscpp 
    sensor_msgs
    tf
    pcl_ros
    pcl_conversions
    pcl_msgs
  INCLUDE_DIRS include
)

include_directories(
  include/edogmapping
  include/edogmapping/motion_update
  include/edogmapping/sensor_update
  include/edogmapping/map
  include/edogmapping/utils
  include/edogmapping/particlefilter
  ${catkin_INCLUDE_DIRS} include
  ${Eigen_INCLUDE_DIRS}
)

add_executable(edogmapping
  src/edogmapping_node.cpp
  src/edogmapping/motion_update/motionmodel.cpp
  src/edogmapping/sensor_update/smmap.cpp
  src/edogmapping/sensor_update/scanmatcher.cpp
  src/edogmapping/utils/stat.cpp
)

target_link_libraries(edogmapping
  ${catkin_LIBRARIES}
)

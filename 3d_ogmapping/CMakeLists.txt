cmake_minimum_required(VERSION 2.8.3)
project(threedogmapping)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  roscpp
  rospy
  std_msgs
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
    rospy
    tf
    pcl_ros
    pcl_conversions
    pcl_msgs
  INCLUDE_DIRS include
)

include_directories(
  include/3d_ogmapping
  include/3d_ogmapping/3d_ogmapper
  include/3d_ogmapping/scanmatcher
  include/3d_ogmapping/grid
  include/3d_ogmapping/utils
  include/3d_ogmapping/particlefilter
  ${catkin_INCLUDE_DIRS} include
  ${Eigen_INCLUDE_DIRS}
)

add_executable(3d_ogmapping
  src/3d_ogmapping_node.cpp
  src/3d_ogmapping/3d_ogmapper/motionmodel.cpp
  src/3d_ogmapping/utils/stat.cpp
  src/3d_ogmapping/scanmatcher/smmap.cpp
  src/3d_ogmapping/scanmatcher/scanmatcher.cpp
)

target_link_libraries(3d_ogmapping
  ${catkin_LIBRARIES}
  )


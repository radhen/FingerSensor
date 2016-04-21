cmake_minimum_required(VERSION 2.8.3)
project(ros_finger_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rviz_visual_tools
  tf
  tf_conversions
  pcl_ros
  pcl_conversions
  sensor_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# baxter control server
add_executable(baxter_control
  src/baxter_control.cpp
)
target_link_libraries(baxter_control
  ${catkin_LIBRARIES}
)

# perception server
add_executable(perception_server 
  src/perception_server.cpp
)
target_link_libraries(perception_server 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)



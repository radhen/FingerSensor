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

# main experiment
add_executable(main_experiment 
  src/main_experiment.cpp
)
target_link_libraries(main_experiment 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )



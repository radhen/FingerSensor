cmake_minimum_required(VERSION 2.8.3)
project(ros_finger_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rviz_visual_tools
  cmake_modules
)

find_package(Eigen REQUIRED)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# main experiment
add_executable(main_experiment src/main_experiment.cpp)
target_link_libraries(main_experiment ${catkin_LIBRARIES})



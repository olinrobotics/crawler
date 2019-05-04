cmake_minimum_required(VERSION 2.8.3)
project(crawler)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  state_controller
  tf2_ros
  rospy
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_to_odom src/publish_tf_as_odom)

target_link_libraries(tf_to_odom ${catkin_LIBRARIES})

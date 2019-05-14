cmake_minimum_required(VERSION 2.8.3)
project(crawler)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  nav_msgs
  geometry_msgs
  state_controller
  tf2_ros
  rospy
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(periodic_snapshotter src/periodic_snapshotter.cpp)
add_executable(tf_to_odom src/publish_tf_as_odom.cpp)
add_executable(hitch_odom src/publish_hitch_odom.cpp)

target_link_libraries(tf_to_odom ${catkin_LIBRARIES})
target_link_libraries(periodic_snapshotter ${catkin_LIBRARIES})
target_link_libraries(hitch_odom ${catkin_LIBRARIES})

add_dependencies( tf_to_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies( periodic_snapshotter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies( hitch_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

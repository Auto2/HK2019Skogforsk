cmake_minimum_required(VERSION 2.8.3)
project(testrig)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  joint_state_publisher
 
  controller_manager
  gazebo_ros
  robot_state_publisher
  roscpp
  rospy
  rviz
  geometry_msgs
  map_msgs
  message_filters
  std_msgs
)

add_compile_options(-std=c++11)

catkin_package(
  INCLUDE_DIRS #include
  CATKIN_DEPENDS roscpp std_msgs

)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor_controller_node src/motor_controller_node.cpp)

add_dependencies(motor_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(motor_controller_node ${catkin_LIBRARIES})

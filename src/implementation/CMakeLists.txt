cmake_minimum_required(VERSION 3.5)
project(implementation)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)



install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)



include_directories(
  include)



add_library(state_publisher_library src/state_publisher.cpp)
ament_target_dependencies(
   state_publisher_library 
   geometry_msgs 
   sensor_msgs
   std_msgs
   rclcpp
   tf2_ros)

add_executable(state_publisher src/state_publisher_node.cpp src/joint.cpp src/parser.cpp src/math_utils.cpp)
ament_target_dependencies(state_publisher rclcpp)

target_link_libraries(state_publisher state_publisher_library)



add_library(cup_simulator_libary src/cup_simulator.cpp)
ament_target_dependencies(
   cup_simulator_libary 
   geometry_msgs 
   sensor_msgs
   std_msgs
   rclcpp
   tf2_ros)

add_executable(cup_simulator src/cup_simulator_node.cpp src/math_utils.cpp)
ament_target_dependencies(cup_simulator rclcpp)

target_link_libraries(cup_simulator cup_simulator_libary)


install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}/)


install(TARGETS
   state_publisher
   cup_simulator
   DESTINATION lib/${PROJECT_NAME})


ament_package()

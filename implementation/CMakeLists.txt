cmake_minimum_required(VERSION 3.5)
project(implementation)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)


add_executable(state_publisher src/state_publisher.cpp)
ament_target_dependencies(
   state_publisher
   geometry_msgs
   sensor_msgs
   rclcpp
   tf2_ros
)


# add_executable(tf2_listener src/tf2_listener.cpp)
# ament_target_dependencies(
#     tf2_listener
#     geometry_msgs
#     rclcpp
#     tf2
#     tf2_ros
#     turtlesim
# )


install(TARGETS
   state_publisher
   DESTINATION lib/${PROJECT_NAME})


# install(TARGETS
#    tf2_listener
#    DESTINATION lib/${PROJECT_NAME})

ament_package()

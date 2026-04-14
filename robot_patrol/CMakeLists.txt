cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Find interface generator dependency
find_package(rosidl_default_generators REQUIRED)

# Generate custom service interfaces (.srv)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"
  DEPENDENCIES sensor_msgs
)


add_executable(patrol_node src/patrol.cpp)
ament_target_dependencies(patrol_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
)

add_executable(direction_service_node src/direction_service.cpp)
ament_target_dependencies(direction_service_node
  rclcpp
  sensor_msgs
)
rosidl_target_interfaces(direction_service_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(test_service_node src/test_service.cpp)
ament_target_dependencies(test_service_node
  rclcpp
  sensor_msgs
)
rosidl_target_interfaces(test_service_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(patrol_with_service_node src/patrol_with_service.cpp)
ament_target_dependencies(patrol_with_service_node rclcpp sensor_msgs geometry_msgs)
rosidl_target_interfaces(patrol_with_service_node ${PROJECT_NAME} "rosidl_typesupport_cpp")


# Install all executables
install(TARGETS
  patrol_node
  direction_service_node
  test_service_node
  patrol_with_service_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch and rviz directories
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
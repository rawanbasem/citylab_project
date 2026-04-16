cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. FIND PACKAGES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED) # Added this!
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. GENERATE INTERFACES (Consolidated into ONE block)
# IMPORTANT: All .srv and .action files must be in the same call
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"
  "action/GoToPose.action"
  DEPENDENCIES sensor_msgs geometry_msgs
)

# 3. EXECUTABLES

# --- Task 2: Action Server ---
add_executable(go_to_pose_action_node src/go_to_pose_action.cpp)
ament_target_dependencies(go_to_pose_action_node 
  rclcpp 
  rclcpp_action 
  geometry_msgs 
  nav_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
)
rosidl_target_interfaces(go_to_pose_action_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# --- Task 1: Patrolling & Service Nodes ---
add_executable(patrol_node src/patrol.cpp)
ament_target_dependencies(patrol_node rclcpp sensor_msgs geometry_msgs tf2)

add_executable(direction_service_node src/direction_service.cpp)
ament_target_dependencies(direction_service_node rclcpp sensor_msgs)
rosidl_target_interfaces(direction_service_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(test_service_node src/test_service.cpp)
ament_target_dependencies(test_service_node rclcpp sensor_msgs)
rosidl_target_interfaces(test_service_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(patrol_with_service_node src/patrol_with_service.cpp)
ament_target_dependencies(patrol_with_service_node rclcpp sensor_msgs geometry_msgs)
rosidl_target_interfaces(patrol_with_service_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 4. INSTALLATION
install(TARGETS
  patrol_node
  direction_service_node
  test_service_node
  patrol_with_service_node
  go_to_pose_action_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
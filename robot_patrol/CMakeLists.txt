cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"
  "action/GoToPose.action"
  DEPENDENCIES sensor_msgs geometry_msgs
)

# Create ament_cmake interface for message and service dependencies
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Add the patrol executable
add_executable(patrol src/patrol.cpp)
target_include_directories(patrol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  patrol
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
)

# Add the direction service executable
add_executable(direction_service src/direction_service.cpp)
target_include_directories(direction_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  direction_service
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
)
target_link_libraries(direction_service "${cpp_typesupport_target}")

# Add the test service executable
add_executable(test_service src/test_service.cpp)
target_include_directories(test_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  test_service
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
)
target_link_libraries(test_service "${cpp_typesupport_target}")

# Add the patrol with service executable
add_executable(patrol_with_service src/patrol_with_service.cpp)
target_include_directories(patrol_with_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  patrol_with_service
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
)
target_link_libraries(patrol_with_service "${cpp_typesupport_target}")

# Add the go to pose action executable
add_executable(go_to_pose_action src/go_to_pose_action.cpp)
target_include_directories(go_to_pose_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  go_to_pose_action
  "rclcpp"
  "rclcpp_action"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_geometry_msgs"
)
target_link_libraries(go_to_pose_action "${cpp_typesupport_target}")

# Install the executables
install(TARGETS 
  patrol
  direction_service
  test_service
  patrol_with_service
  go_to_pose_action
  DESTINATION lib/${PROJECT_NAME})

# Install the launch directory
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
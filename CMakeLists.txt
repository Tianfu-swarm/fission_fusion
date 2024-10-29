cmake_minimum_required(VERSION 3.5)
project(information_dissemination)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces for custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Position.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Include directories for generated message headers
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/include
)

# Add the executable
add_executable(${PROJECT_NAME}_node src/controller.cpp)

# Link dependencies to the executable
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  rosidl_default_runtime
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install the target
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
# Package configuration
ament_package()

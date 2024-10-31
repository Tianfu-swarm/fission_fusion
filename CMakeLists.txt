cmake_minimum_required(VERSION 3.5)
project(information_dissemination)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


# Include directories for generated message headers
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/include
)

# Add the executable
add_executable(controller src/controller.cpp)

# Link dependencies to the executable
ament_target_dependencies(controller
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_geometry_msgs
  angles
  
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install the target
install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)

# Package configuration
ament_package()

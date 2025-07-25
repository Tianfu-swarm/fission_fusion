cmake_minimum_required(VERSION 3.5)
project(fission_fusion)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(angles REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)



# Include directories for generated message headers
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/include
)

# Add the executable
add_executable(fission_fusion src/P_controller.cpp 
                              src/visualization.cpp 
                              src/SDRM_controller.cpp 
                              src/main.cpp 
                              src/skybat_controller.cpp
                              src/SFFM.cpp
                              src/extrema_propagation.cpp)

# Link dependencies to the executable
ament_target_dependencies(fission_fusion
  rclcpp
  std_msgs
  yaml-cpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  angles
  sensor_msgs
)

target_link_libraries(fission_fusion yaml-cpp)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
#Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data)


configure_file(${CMAKE_SOURCE_DIR}/config/config.yaml ${CMAKE_BINARY_DIR}/config/config.yaml COPYONLY)

# Install the target
install(TARGETS
  fission_fusion
  DESTINATION lib/${PROJECT_NAME}
)

# Package configuration
ament_package()

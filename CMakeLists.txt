cmake_minimum_required(VERSION 3.5)
project(information_dissemination)

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 创建可执行文件
add_executable(information_dissemination src/controller.cpp)

# 链接依赖
ament_target_dependencies(information_dissemination rclcpp std_msgs)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装目标
install(TARGETS
information_dissemination
  DESTINATION lib/${PROJECT_NAME}
)

# 导出包
ament_package()

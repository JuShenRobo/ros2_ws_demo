cmake_minimum_required(VERSION 3.8)
project(grasp_topic)

# 默认使用 C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ========== 新增这一行（关键） ==========
find_package(ament_cmake_python REQUIRED)  # 启用Python可执行文件注册
# =======================================

# 声明消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Camera.msg"
  "msg/Grasp.msg"
  "msg/Plan.msg"
  "msg/Pose.msg"
  DEPENDENCIES std_msgs
)

# 导出依赖
ament_export_dependencies(rclcpp rclpy std_msgs)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# ========== 新增这一行（可选，确保Python依赖传递） ==========
ament_export_dependencies(ament_cmake_python)
# =======================================

# 添加包含目录
include_directories(
  include
)

# 如果需要声明C++库
# add_library(${PROJECT_NAME} src/${PROJECT_NAME}/grasp_topic.cpp)
# ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs)

# 添加可执行文件（如果需要）
# add_executable(grasp_node src/grasp_node.cpp)
# ament_target_dependencies(grasp_node rclcpp std_msgs)
# target_link_libraries(grasp_node ${PROJECT_NAME})

# 安装可执行文件
# install(TARGETS grasp_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# 安装库文件
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装Python脚本（如果有）
install(PROGRAMS
  scripts/grasp_duco_ros2.py
  scripts/xinhai_connect.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件（如果有）
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

# 必须在文件结尾调用
ament_package()
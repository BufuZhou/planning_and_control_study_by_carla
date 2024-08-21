cmake_minimum_required(VERSION 3.5)
project(localization)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# clangd 自动在build目录下生成compile_commands.json文件
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(common_msgs REQUIRED)

add_executable(${PROJECT_NAME}_node
src/localization_main.cpp
src/localization_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp nav_msgs tf2_geometry_msgs common_msgs)
target_include_directories( ${PROJECT_NAME}_node PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)
# 安装可执行文件到install文件中
install(TARGETS   ${PROJECT_NAME}_node
  # intall文件中的目录install/control/lib/control
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

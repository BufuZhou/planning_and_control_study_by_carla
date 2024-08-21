cmake_minimum_required(VERSION 3.5)
project(planning)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

message("CMAKE_COMPILER_IS_GNUCXX: ${CMAKE_COMPILER_IS_GNUCXX}")
message(${CMAKE_CXX_COMPILER_ID})
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# clangd 自动在build目录下生成compile_commands.json文件
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
# ros相关的依赖，通过ament_target_dependencies配置
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_msgs REQUIRED)

add_executable(${PROJECT_NAME}_node
src/planning_main.cpp
src/planning_node.cpp
)
# 添加ros相关的依赖，和前面find_package相对应
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs common_msgs)

# 添加编译可执行程序或者库所需的头文件路径
target_include_directories( ${PROJECT_NAME}_node PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

# 读取yaml配置参数
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY param
        DESTINATION share/${PROJECT_NAME}/
)


# 安装可执行文件到install文件中
install(TARGETS   ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

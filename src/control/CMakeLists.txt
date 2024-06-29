cmake_minimum_required(VERSION 3.5)
project(control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# ros相关的依赖，通过ament_target_dependencies配置
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
include_directories(${EIGEN3_INCLUDE_DIR})
message(${EIGEN3_INCLUDE_DIR})
if (NOT Eigen3)
  message("Eigen3 not found")
endif()


add_executable(${PROJECT_NAME}_node
src/control_main.cpp
src/control_node.cpp
src/lat_controller.cpp
src/digital_filter_coefficients.cpp
src/digital_filter.cpp
src/mean_filter.cpp
src/interpolation_1d.cpp
src/trajectory_analyzer.cpp
)
# 添加ros相关的依赖，和前面find_package相对应
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs carla_msgs common_msgs Eigen3)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

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
  # intall文件中的目录install/control/lib/control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.5)
project(control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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

# protobuf
find_package(Protobuf REQUIRED)
if (PROTOBUF_FOUND)
    message("protobuf found")
else ()
    message(FATAL_ERROR "Cannot find Protobuf")
endif ()

# 输出cmake当前源文件路径
# message("CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
# 输出cmake编译文件路径
# message("CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")

# protobuf
find_package(Protobuf REQUIRED)
if (PROTOBUF_FOUND)
    message("protobuf found")
else ()
    message(FATAL_ERROR "Cannot find Protobuf")
endif ()

# 设置proto所在目录
SET(PROTO_FILE_PATH ${CMAKE_SOURCE_DIR}/proto)
message("PROTO_FILE_PATH: ${PROTO_FILE_PATH}")

# 设置proto生成文件路径
set(PROTO_OUTPUT_PATH  ${CMAKE_BINARY_DIR}/control_proto_generate)
if(IS_DIRECTORY ${PROTO_OUTPUT_PATH})
  message("PROTO_OUTPUT_PATH exist.")
else()
  file(MAKE_DIRECTORY ${PROTO_OUTPUT_PATH})
  message("PROTO_OUTPUT_PATH generate.")
endif()
message("PROTO_OUTPUT_PATH: ${PROTO_OUTPUT_PATH}")

# 获取需要编译的proto文件
file(GLOB_RECURSE ALL_PROTO_FILES ${PROTO_FILE_PATH}/*.proto)
message("ALL_PROTO_FILE: ${ALL_PROTO_FILES}")

# 根据proto编译.cc和.hpp
set(PROTO_GENERATE_SRC "")
set(PROTO_GENERATE_HDRS "")
foreach(PROTO_FILE ${ALL_PROTO_FILES})
   get_filename_component(FILE_NAME ${PROTO_FILE} NAME_WE)
   list(APPEND PROTO_GENERATE_HDRS "${PROTO_OUTPUT_PATH}/${FILE_NAME}.pb.hpp") 
   list(APPEND PROTO_GENERATE_SRC "${PROTO_OUTPUT_PATH}/${FILE_NAME}.pb.cc")
   message("GENERATE PROTO HEADER FILES: ${PROTO_GENERATE_HDRS}")
   message("GENERATE PROTO SOURCE FILES: ${PROTO_GENERATE_SRC}")
   # 使用自定义命令
   add_custom_command(
      OUTPUT 
         "${PROTO_OUTPUT_PATH}/${FILE_NAME}.pb.hpp"
         "${PROTO_OUTPUT_PATH}/${FILE_NAME}.pb.cc"
      COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS 
         --cpp_out  ${PROTO_OUTPUT_PATH}
         -I ${PROTO_FILE_PATH}
         ${PROTO_FILE}
      DEPENDS ${PROTO_FILE}
      COMMENT "Running C++ protocol buffer compiler on ${PROTO_FILE}"
      VERBATIM
   )
endforeach()
set_source_files_properties(${PROTO_GENERATE_SRC} ${PROTO_GENERATE_HDRS} PROPERTIES GENERATED TRUE)
# 配合自定义命令生成proto对应的代码
add_custom_target(generate_message ALL
                DEPENDS ${PROTO_GENERATE_SRC} ${PROTO_GENERATE_HDRS}
                COMMENT "generate message target"
                VERBATIM
                )

add_executable(${PROJECT_NAME}_node
src/control_main.cpp
src/control_node.cpp
src/lat_controller.cpp
src/digital_filter_coefficients.cpp
src/digital_filter.cpp
src/mean_filter.cpp
src/interpolation_1d.cpp
src/trajectory_analyzer.cpp
${PROTO_OUTPUT_PATH}
)

# 添加ros相关的依赖，和前面find_package相对应
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs carla_msgs common_msgs Eigen3)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

target_include_directories( ${PROJECT_NAME}_node PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
                            $<BUILD_INTERFACE:${Protobuf_INCLUDE_DIRS}>
                            $<BUILD_INTERFACE:${PROTO_OUTPUT_PATH}>
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

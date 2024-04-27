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

# protobuf
find_package(Protobuf REQUIRED)
if (PROTOBUF_FOUND)
    message("protobuf found")
else ()
    message(FATAL_ERROR "Cannot find Protobuf")
endif ()

# 输出cmake当前源文件路径
message("CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")

# 输出cmake编译文件路径
message("CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")

# 设置proto所在目录
SET(PROTO_FILE_PATH ${CMAKE_SOURCE_DIR}/common_msgs)
message("PROTO_FILE_PATH: ${PROTO_FILE_PATH}")

# 设置proto生成文件路径
set(PROTO_OUTPUT_PATH  ${CMAKE_BINARY_DIR}/common_msgs_generate)
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
# 配合自定命令add_custom_command生成proto对应的代码
add_custom_target(${PROJECT_NAME}_generate_proto ALL
                DEPENDS ${PROTO_GENERATE_SRC} ${PROTO_GENERATE_HDRS}
                COMMENT "generate message target"
                VERBATIM
                )

# add_executable(${PROJECT_NAME} src/control_main.cpp ${PROTO_OUTPUT_PATH})
# 设定本模块的可执行程序
add_executable(${PROJECT_NAME})
# 添加ros相关的依赖，和前面find_package相对应
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs)

# 添加编译可执行程序或者库所需的头文件路径
target_include_directories(${PROJECT_NAME}
  PRIVATE
  # protobuf库相关的头文件
  $<BUILD_INTERFACE:${Protobuf_INCLUDE_DIRS}>
  # 当前模块所需proto生成代码的文件
  $<BUILD_INTERFACE:${PROTO_OUTPUT_PATH}>
)

# 添加编译可执行程序或者库所需的源文件路径
target_sources(${PROJECT_NAME} 
            PRIVATE
            src/control_main.cpp 
            ${PROTO_GENERATE_SRC}
            )
# 添加编译可执行程序或者库所需的链接库
target_link_libraries(${PROJECT_NAME}
            ${Protobuf_LIBRARIES}
        )

# 安装可执行文件到install文件中
install(TARGETS   ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

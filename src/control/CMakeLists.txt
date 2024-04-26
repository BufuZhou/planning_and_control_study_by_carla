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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
set_source_files_properties(${PROTO_GENERATE_SRC} ${PROTO_GENERATE_HDRS} PROPERTIES GENERATED TRUE)
# 配合自定义命令生成proto对应的代码
add_custom_target(generate_message ALL
                DEPENDS ${PROTO_GENERATE_SRC} ${PROTO_GENERATE_HDRS}
                COMMENT "generate message target"
                VERBATIM
                )

add_executable(control src/control_main.cpp ${PROTO_OUTPUT_PATH})
target_include_directories(control
  PRIVATE
  $<BUILD_INTERFACE:${Protobuf_INCLUDE_DIRS}>
  $<BUILD_INTERFACE:${PROTO_OUTPUT_PATH}>
)
target_sources(control 
            PRIVATE
            src/control_main.cpp 
            ${PROTO_GENERATE_SRC}
            )
target_link_libraries(control
            ${Protobuf_LIBRARIES}
        )





ament_target_dependencies(control rclcpp std_msgs)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

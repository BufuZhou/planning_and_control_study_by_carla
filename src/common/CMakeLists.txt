cmake_minimum_required(VERSION 3.5)
project(common)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# clangd 自动在build目录下生成compile_commands.json文件
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(ament_cmake_auto REQUIRED) 
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED 
    include/common/proto_util.hpp
    src/proto_util.cpp 
    )

ament_auto_package ()

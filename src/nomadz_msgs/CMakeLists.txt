cmake_minimum_required(VERSION 3.16)

project(nomadz_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files "msg/SystemStats.msg")

if(${CMAKE_CROSSCOMPILING})
  rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files})
else()
  rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} ADD_LINTER_TESTS)
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()

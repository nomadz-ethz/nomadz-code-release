cmake_minimum_required(VERSION 3.16)
project(nomadz_modeling_msgs)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nomadz_vision_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/BallModel.msg"
  "msg/RegisteredLines.msg"
  "msg/RobotPose.msg"
  "msg/RobotPoseList.msg"
  "msg/WorldModel.msg"
  DEPENDENCIES
  geometry_msgs
  nomadz_vision_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_export_include_directories("include/${PROJECT_NAME}")

ament_package()

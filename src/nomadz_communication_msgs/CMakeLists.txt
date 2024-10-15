cmake_minimum_required(VERSION 3.16)
project(nomadz_communication_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nomadz_modeling_msgs REQUIRED)
find_package(nomadz_behavior_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/RobocupGameControlData.msg"
  "msg/RobocupGameControlReturnData.msg"
  "msg/RobotInfo.msg"
  "msg/TeamCommData.msg"
  "msg/TeamCommInfo.msg"
  "msg/TeamInfo.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nomadz_modeling_msgs
  nomadz_behavior_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories("include/${PROJECT_NAME}")

ament_package()

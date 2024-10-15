cmake_minimum_required(VERSION 3.5)
project(nao_lola_command_msgs)

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
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/ChestLed.msg"
  "msg/HeadLeds.msg"
  "msg/JointIndexes.msg"
  "msg/JointRequests.msg"
  "msg/EarLeds.msg"
  "msg/EyeLeds.msg"
  "msg/FootLed.msg"
  "msg/SonarUsage.msg"
  DEPENDENCIES
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

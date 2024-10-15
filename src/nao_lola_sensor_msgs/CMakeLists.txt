cmake_minimum_required(VERSION 3.5)
project(nao_lola_sensor_msgs)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/Imu.msg"
  "msg/Battery.msg"
  "msg/Buttons.msg"
  "msg/Fsr.msg"
  "msg/JointIndexes.msg"
  "msg/JointData.msg"
  "msg/RobotConfig.msg"
  "msg/Sonar.msg"
  DEPENDENCIES
  geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime geometry_msgs)

ament_package()

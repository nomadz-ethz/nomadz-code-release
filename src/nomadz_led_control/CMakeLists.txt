cmake_minimum_required(VERSION 3.16)
project(nomadz_led_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_CROSSCOMPILING)
  add_compile_definitions(TARGET_ROBOT)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nao_lola_command_msgs REQUIRED)
find_package(nomadz_motion_control_msgs REQUIRED)
find_package(nomadz_communication_msgs REQUIRED)
find_package(nomadz_modeling_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# build nomadz_led_control library
add_library(nomadz_led_control SHARED src/led_handler.cpp)

target_include_directories(
  nomadz_led_control
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
)

# link the dependencies
ament_target_dependencies(
  nomadz_led_control
  rclcpp
  rclcpp_components
  nao_lola_command_msgs
  nomadz_motion_control_msgs
  nomadz_communication_msgs
  nomadz_modeling_msgs
  std_msgs
)

rclcpp_components_register_node(
  nomadz_led_control
  PLUGIN "nomadz_led_control::LEDHandler"
  EXECUTABLE led_handler
)

install(DIRECTORY include/ DESTINATION include/)

install(
  TARGETS nomadz_led_control
  EXPORT nomadz_led_controlTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()

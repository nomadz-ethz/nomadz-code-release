cmake_minimum_required(VERSION 3.16)

project(example_nao_lola_control)

find_package(ament_cmake)

find_package(rclcpp REQUIRED)
find_package(nomadz_definitions REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)
find_package(nao_lola_command_msgs REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(example_nao_lola_control src/example_nao_lola_control.cpp)

ament_target_dependencies(
  example_nao_lola_control
  PUBLIC nomadz_definitions nao_lola_sensor_msgs nao_lola_command_msgs rclcpp
)

install(TARGETS example_nao_lola_control DESTINATION lib/${PROJECT_NAME})

ament_package()

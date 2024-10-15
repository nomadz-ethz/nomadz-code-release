cmake_minimum_required(VERSION 3.16)

project(nomadz_communication)

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nomadz_core REQUIRED)
find_package(nomadz_communication_msgs REQUIRED)
find_package(nomadz_configuration REQUIRED)
find_package(nomadz_modeling_msgs REQUIRED)
find_package(nomadz_proprioception_msgs REQUIRED)
find_package(
  Boost
  COMPONENTS system
  REQUIRED
)

add_library(
  nomadz_communication SHARED src/udp_communicator.cpp src/ros_conversion.cpp
)

target_include_directories(
  nomadz_communication
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  nomadz_communication
  SYSTEM
  rclcpp
  rclcpp_components
  geometry_msgs
  Boost
)

ament_target_dependencies(
  nomadz_communication
  nomadz_core
  nomadz_communication_msgs
  nomadz_configuration
  nomadz_modeling_msgs
  nomadz_proprioception_msgs
)

rclcpp_components_register_node(
  nomadz_communication
  PLUGIN "nomadz_communication::UDPCommunicator"
  EXECUTABLE udp_communicator_node
)

install(DIRECTORY include/ DESTINATION include/)

install(
  TARGETS nomadz_communication
  EXPORT nomadz_communicationTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()

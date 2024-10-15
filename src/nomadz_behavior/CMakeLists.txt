cmake_minimum_required(VERSION 3.16)

project(nomadz_behavior)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_CROSSCOMPILING)
  add_compile_definitions(TARGET_ROBOT)
endif()

find_package(ament_cmake)
find_package(spdlog REQUIRED)
find_package(behaviortree_cpp)
find_package(nomadz_core REQUIRED)
find_package(nomadz_behavior_msgs REQUIRED)
find_package(nomadz_communication_msgs REQUIRED)
find_package(nomadz_motion_control_msgs REQUIRED)
find_package(nomadz_proprioception_msgs REQUIRED)
find_package(nomadz_modeling_msgs REQUIRED)
find_package(nomadz_audio_processing_msgs REQUIRED)
find_package(nomadz_configuration REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

set(action_nodes
    src/actions.cpp
    src/action_nodes/motion_nodes/walk_actions.cpp
    src/action_nodes/motion_nodes/special_actions.cpp
    src/action_nodes/motion_nodes/head_motion_action.cpp
    src/action_nodes/helper_nodes/compute_positions.cpp
    src/action_nodes/helper_nodes/state_provider.cpp
    src/action_nodes/high_level_actions.cpp
)

set(check_nodes src/checks.cpp src/check_nodes/check_nodes.cpp)

add_executable(
  behavior_node
  src/main.cpp
  src/behavior.cpp
  src/ros_interface.cpp
  ${action_nodes}
  ${check_nodes}
  src/helper/ros_conversion.cpp
  src/helper/common_functions.cpp
  src/helper/target_pose_generator.cpp
  src/helper/path_planner.cpp
  src/helper/bt_enums.cpp
)

target_include_directories(
  behavior_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  behavior_node SYSTEM rclcpp behaviortree_cpp ament_index_cpp
)

ament_target_dependencies(
  behavior_node
  nomadz_core
  nomadz_behavior_msgs
  nomadz_modeling_msgs
  nomadz_motion_control_msgs
  nomadz_communication_msgs
  nomadz_proprioception_msgs
  nomadz_configuration
  geometry_msgs
)

target_link_libraries(behavior_node spdlog::spdlog yaml-cpp)

install(TARGETS behavior_node DESTINATION lib/${PROJECT_NAME})

add_library(
  behavior_helper SHARED
  src/helper/ego_status_provider.cpp
  src/helper/game_status_provider.cpp
  src/helper/ros_conversion.cpp
)

target_include_directories(
  behavior_helper
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(behavior_helper SYSTEM rclcpp rclcpp_components)

target_link_libraries(behavior_helper Eigen3::Eigen)

ament_target_dependencies(
  behavior_helper
  nomadz_core
  nomadz_configuration
  nomadz_modeling_msgs
  nomadz_behavior_msgs
  nomadz_motion_control_msgs
  nomadz_communication_msgs
  nomadz_proprioception_msgs
  nomadz_audio_processing_msgs
  geometry_msgs
)
rclcpp_components_register_node(
  behavior_helper
  PLUGIN "nomadz_behavior::EgoStatusProvider"
  EXECUTABLE ego_status_provider
)

rclcpp_components_register_node(
  behavior_helper
  PLUGIN "nomadz_behavior::GameStatusProvider"
  EXECUTABLE game_status_provider
)

install(DIRECTORY include/ DESTINATION include/)

install(
  TARGETS behavior_helper
  EXPORT behavior_helperTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()

cmake_minimum_required(VERSION 3.16)
project(nomadz_modeling)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(nomadz_configuration REQUIRED)
find_package(nomadz_core REQUIRED)
find_package(nomadz_behavior_msgs REQUIRED)

find_package(nomadz_modeling_msgs REQUIRED)
find_package(nomadz_motion_control_msgs REQUIRED)
find_package(nomadz_vision_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost REQUIRED)
find_package(nomadz_communication_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_geometry REQUIRED)

generate_parameter_library(
  self_locator_parameters config/self_locator_params.yaml
)
generate_parameter_library(
  self_world_model_params config/self_world_model_params.yaml
)

add_library(
  ${PROJECT_NAME} SHARED
  src/self_locator.cpp
  src/line_registrator.cpp
  src/landmark_registrator.cpp
  src/pose_registrator.cpp
  src/self_locator_utils.cpp
  src/particle.cpp
  src/ros_conversion.cpp
  src/world_model_node.cpp
  src/ball_locator.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  self_locator_parameters
  self_world_model_params
  spdlog::spdlog
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "nomadz_modeling::WorldModelNode"
  EXECUTABLE modeling_node
)

ament_target_dependencies(
  ${PROJECT_NAME} SYSTEM image_geometry rclcpp rclcpp_components
)
ament_target_dependencies(
  ${PROJECT_NAME}
  sensor_msgs
  nomadz_core
  nomadz_configuration
  nomadz_behavior_msgs
  nomadz_modeling_msgs
  nomadz_motion_control_msgs
  nomadz_vision_msgs
  nomadz_communication_msgs
  Boost
)
target_link_libraries(
  ${PROJECT_NAME} self_locator_parameters self_world_model_params Eigen3::Eigen
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_line_registration test/test_line_registration.cpp)
  target_include_directories(
    test_line_registration
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_line_registration nomadz_core)
  target_link_libraries(test_line_registration ${PROJECT_NAME})

  ament_add_gtest(test_pose_registration test/test_pose_registration.cpp)
  target_include_directories(
    test_pose_registration
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_pose_registration nomadz_core)
  target_link_libraries(test_pose_registration ${PROJECT_NAME})

  ament_add_gtest(
    test_landmark_registration test/test_landmark_registration.cpp
  )
  target_include_directories(
    test_landmark_registration
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_landmark_registration nomadz_core)
  target_link_libraries(test_landmark_registration ${PROJECT_NAME})
endif()

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_package()

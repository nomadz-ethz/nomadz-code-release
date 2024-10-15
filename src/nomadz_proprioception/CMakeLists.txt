cmake_minimum_required(VERSION 3.16)

project(nomadz_proprioception)

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
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nomadz_definitions REQUIRED)
find_package(nomadz_kinematics REQUIRED)
find_package(nomadz_core REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)
find_package(nomadz_proprioception_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(
  nomadz_proprioception SHARED
  src/sensor_data_filter.cpp
  src/fsr_calibrator.cpp
  src/foot_support_estimator.cpp
  src/ros_conversion.cpp
  src/transforms.cpp
  src/proprioception_node.cpp
  src/fall_down_state_detector.cpp
  src/arm_contact_estimator.cpp
)

target_include_directories(
  nomadz_proprioception
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  nomadz_proprioception
  SYSTEM
  rclcpp
  rclcpp_components
  geometry_msgs
  Boost
  message_filters
  tf2_ros
  tf2_eigen
)

target_link_libraries(nomadz_proprioception Eigen3::Eigen)

ament_target_dependencies(
  nomadz_proprioception
  nomadz_definitions
  nao_lola_sensor_msgs
  nomadz_kinematics
  nomadz_core
  nomadz_proprioception_msgs
)

rclcpp_components_register_node(
  nomadz_proprioception
  PLUGIN "nomadz_proprioception::ProprioceptionNode"
  EXECUTABLE proprioception_node
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/)

install(
  TARGETS nomadz_proprioception
  EXPORT nomadz_proprioceptionTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()

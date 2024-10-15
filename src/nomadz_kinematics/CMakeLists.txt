cmake_minimum_required(VERSION 3.16)

project(nomadz_kinematics)

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
find_package(nomadz_definitions REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)
find_package(nomadz_proprioception_msgs REQUIRED)
find_package(nomadz_core REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(
  nomadz_kinematics SHARED
  src/inverse_kinematics.cpp
  src/forward_kinematics.cpp
  src/mass_calibration_parser.cpp
  src/robot_model.cpp
  src/ros_conversion.cpp
)

target_include_directories(
  nomadz_kinematics
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nomadz_kinematics SYSTEM ament_index_cpp tf2_eigen)
ament_target_dependencies(
  nomadz_kinematics
  nomadz_definitions
  nao_lola_sensor_msgs
  nomadz_proprioception_msgs
  nomadz_core
)

target_link_libraries(nomadz_kinematics Eigen3::Eigen yaml-cpp)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(
  TARGETS nomadz_kinematics
  EXPORT nomadz_kinematicsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)

ament_export_targets(nomadz_kinematicsTargets HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    test_kinematics
    test/test_inverse_kinematics.cpp
    test/test_forward_kinematics.cpp
  )
  target_link_libraries(test_kinematics nomadz_kinematics Eigen3::Eigen)
endif()

if(NOT CMAKE_CROSSCOMPILING)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(nao_lola_command_msgs REQUIRED)

  add_executable(nomadz_kinematics_node test/kinematics_tester.cpp)

  ament_target_dependencies(nomadz_kinematics_node SYSTEM rclcpp geometry_msgs)
  ament_target_dependencies(
    nomadz_kinematics_node nomadz_definitions nao_lola_command_msgs
  )

  target_link_libraries(nomadz_kinematics_node nomadz_kinematics)

  install(TARGETS nomadz_kinematics_node DESTINATION lib/${PROJECT_NAME})

  add_executable(robot_model_test_node src/robot_model_test_node.cpp)
  ament_target_dependencies(
    robot_model_test_node
    rclcpp
    visualization_msgs
    nao_lola_command_msgs
    nao_lola_sensor_msgs
  )

  target_link_libraries(robot_model_test_node nomadz_kinematics Eigen3::Eigen)
  install(TARGETS robot_model_test_node DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_dependencies(Eigen3 nomadz_definitions tf2_eigen nomadz_core)

ament_package()

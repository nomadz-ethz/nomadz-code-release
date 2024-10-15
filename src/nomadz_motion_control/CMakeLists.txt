cmake_minimum_required(VERSION 3.16)

project(nomadz_motion_control)

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
set(PACKAGE_SYSTEM_DEPENDENCIES
    ament_cmake ament_index_cpp rclcpp rclcpp_components tf2_eigen
)

set(PACKAGE_ROS_DEPENDENCIES
    nomadz_core
    nomadz_definitions
    nomadz_kinematics
    std_msgs
    geometry_msgs
    visualization_msgs
    nao_lola_command_msgs
    nao_lola_sensor_msgs
    nomadz_behavior_msgs
    nomadz_modeling_msgs
    nomadz_proprioception_msgs
    nomadz_motion_control_msgs
)
set(ALL_PACKAGE_DEPENDENCIES
    ${PACKAGE_SYSTEM_DEPENDENCIES}
    ${PACKAGE_ROS_DEPENDENCIES}
    Eigen3
    yaml-cpp
    Boost
    generate_parameter_library
)

foreach(pkg ${ALL_PACKAGE_DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

set(PARAMS
    walk_engine_parameters
    arm_motion_engine_parameters
    head_engine_parameters
    special_action_generator_parameters
)

foreach(param ${PARAMS})
  generate_parameter_library(${param} config/ros_params/${param}.yaml)
endforeach()

file(
  GLOB_RECURSE
  MOTION_CONTROL_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

add_library(nomadz_motion_control SHARED ${MOTION_CONTROL_SOURCE_FILES})

target_include_directories(
  nomadz_motion_control
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  nomadz_motion_control SYSTEM ${PACKAGE_SYSTEM_DEPENDENCIES}
)

ament_target_dependencies(nomadz_motion_control ${PACKAGE_ROS_DEPENDENCIES})

rclcpp_components_register_node(
  nomadz_motion_control
  PLUGIN "nomadz_motion_control::MotionControlNode"
  EXECUTABLE motion_control_node
)

target_link_libraries(nomadz_motion_control ${PARAMS} yaml-cpp Eigen3::Eigen)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_interpolation test/test_interpolation.cpp)

  target_link_libraries(test_interpolation nomadz_motion_control)
endif()

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(
  TARGETS nomadz_motion_control
  EXPORT nomadz_motion_controlTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)

if(NOT CMAKE_CROSSCOMPILING)
  add_executable(kick_tester test/kick_tester.cpp)
  target_include_directories(
    kick_tester PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(
    kick_tester
    rclcpp
    visualization_msgs
    nao_lola_command_msgs
    nao_lola_sensor_msgs
    nomadz_kinematics
    nomadz_behavior_msgs
    nomadz_motion_control_msgs
  )
  target_link_libraries(kick_tester nomadz_motion_control)
  install(TARGETS kick_tester DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()

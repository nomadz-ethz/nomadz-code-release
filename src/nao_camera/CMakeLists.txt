cmake_minimum_required(VERSION 3.16)

project(nao_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_CROSSCOMPILING)
  add_compile_definitions(TARGET_ROBOT)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

if(NOT CMAKE_CROSSCOMPILING)
  find_package(
    Boost
    COMPONENTS system
    REQUIRED
  )
endif()

add_library(
  nao_camera SHARED
  src/v4l2_device_interface.cpp
  src/nao_camera_device.cpp
  src/parameters.cpp
  src/camera_device_factory.cpp
  src/nao_camera.cpp
  src/dummy_image_processor.cpp
)

# include simulated and fake camera device only if not cross-compiling
if(NOT CMAKE_CROSSCOMPILING)
  target_sources(
    nao_camera
    PRIVATE src/fake_camera_device.cpp src/simulated_camera_device.cpp
  )
endif()

target_include_directories(
  nao_camera
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
)

set(system_deps
    rclcpp rclcpp_components sensor_msgs image_transport camera_info_manager
)
if(NOT CMAKE_CROSSCOMPILING)
  list(APPEND system_deps Boost)
endif()

ament_target_dependencies(nao_camera SYSTEM ${system_deps})

rclcpp_components_register_node(
  nao_camera
  PLUGIN "nao_camera::NaoCamera"
  EXECUTABLE "nao_camera_node"
)

rclcpp_components_register_node(
  nao_camera
  PLUGIN "nao_camera::DummyImageProcessor"
  EXECUTABLE "dummy_image_processor_node"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gmock(test_nao_camera_device test/unit/test_nao_camera_device.cpp)

  # suppress warnings from the versions of gtest / gmock shipped with ROS 2
  target_compile_options(gmock PUBLIC -Wno-deprecated-copy)
  target_compile_options(gmock_main PUBLIC -Wno-deprecated-copy)

  target_compile_options(
    test_nao_camera_device PRIVATE -Wno-gnu-zero-variadic-macro-arguments
  )

  target_link_libraries(test_nao_camera_device nao_camera)

  add_launch_test(test/integration/nao_camera_cyclic_pipeline_test.py)
endif()

# install

install(
  DIRECTORY include/
  DESTINATION include/
  PATTERN "${include_install_exclude_pattern}" EXCLUDE
)

install(
  TARGETS nao_camera
  EXPORT nao_cameraTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(CMAKE_CROSSCOMPILING)
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "sim_*" EXCLUDE
  )
  install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
    PATTERN "sim_*" EXCLUDE
  )
else()
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()

ament_export_dependencies(rclcpp rclcpp_components sensor_msgs image_transport)

if(NOT CMAKE_CROSSCOMPILING)
  ament_export_dependencies(Boost)
endif()

ament_export_targets(nao_cameraTargets HAS_LIBRARY_TARGET)

ament_export_libraries(nao_camera)

ament_package()

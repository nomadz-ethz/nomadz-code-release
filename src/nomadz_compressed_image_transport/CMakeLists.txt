cmake_minimum_required(VERSION 3.16)
project(nomadz_compressed_image_transport)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(JPEG REQUIRED)

add_library(
  ${PROJECT_NAME} SHARED
  src/compressed_publisher.cpp src/compressed_subscriber.cpp src/manifest.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  image_transport
  sensor_msgs
  pluginlib
  JPEG
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
pluginlib_export_plugin_description_file(
  image_transport nomadz_compressed_plugins.xml
)

ament_package()

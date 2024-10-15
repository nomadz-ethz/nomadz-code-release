cmake_minimum_required(VERSION 3.16)
project(nao_lola_client)

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

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)
find_package(nao_lola_command_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(nomadz_core REQUIRED)
find_package(
  Boost
  COMPONENTS system
  REQUIRED
)
find_package(msgpack REQUIRED)
find_package(sensor_msgs REQUIRED)

# build msgpack_parser_lib
add_library(msgpack_parser_lib SHARED src/msgpack_parser.cpp)

ament_target_dependencies(msgpack_parser_lib SYSTEM rclcpp Boost msgpack)
ament_target_dependencies(msgpack_parser_lib nao_lola_sensor_msgs)

# build msgpack_packer_lib
add_library(msgpack_packer_lib SHARED src/msgpack_packer.cpp)

ament_target_dependencies(msgpack_packer_lib SYSTEM rclcpp Boost msgpack)
ament_target_dependencies(msgpack_packer_lib nao_lola_command_msgs)

generate_parameter_library(
  nao_lola_client_parameters config/nao_lola_client_parameters.yaml
)

# build nao_lola_client
add_executable(
  nao_lola_client
  src/nao_lola_client_node.cpp
  src/nao_lola_client.cpp
  src/connection.cpp
  src/conversion.cpp
)

ament_target_dependencies(nao_lola_client std_msgs sensor_msgs nomadz_core)

target_link_libraries(
  nao_lola_client
  msgpack_parser_lib
  msgpack_packer_lib
  nao_lola_client_parameters
)

# Install
install(TARGETS nao_lola_client DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(
  TARGETS msgpack_parser_lib msgpack_packer_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

endif()

ament_package()

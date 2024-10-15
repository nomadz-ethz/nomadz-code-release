cmake_minimum_required(VERSION 3.16)
project(nomadz_configuration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(nomadz_configuration src/io.cpp src/field_info.cpp)

target_include_directories(
  nomadz_configuration
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nomadz_configuration SYSTEM ament_index_cpp)

target_link_libraries(nomadz_configuration Eigen3::Eigen yaml-cpp)

if(CMAKE_CROSSCOMPILING)
  target_compile_definitions(nomadz_configuration PRIVATE TARGET_ROBOT)
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(
  TARGETS nomadz_configuration
  EXPORT nomadz_configurationTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES
  DESTINATION include
)

ament_export_targets(nomadz_configurationTargets HAS_LIBRARY_TARGET)

ament_export_libraries(nomadz_configuration)

ament_export_dependencies(Eigen3 yaml-cpp)

ament_package()

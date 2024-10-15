cmake_minimum_required(VERSION 3.16)

project(nomadz_audio_processing)

include(cmake/download_model.cmake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_CROSSCOMPILING)
  add_compile_definitions(TARGET_ROBOT)
endif()

set(MODEL_DOWNLOAD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/models)
download_model(whistle/whistle-recognizer-fast-20230630.tflite)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nomadz_behavior_msgs REQUIRED)
find_package(nomadz_configuration REQUIRED)
find_package(nomadz_communication_msgs REQUIRED)
find_package(nomadz_audio_processing_msgs REQUIRED)

find_package(tensorflowlite REQUIRED)
find_package(FFTW3 REQUIRED)

# core library
add_library(
  nomadz_audio_processing_core
  src/audio_provider.cpp src/whistle_recognizer.cpp src/libmfcc.cpp
)

target_include_directories(
  nomadz_audio_processing_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nomadz_audio_processing_core SYSTEM rclcpp)
if(CMAKE_CROSSCOMPILING)
  find_package(ALSA REQUIRED)
  ament_target_dependencies(nomadz_audio_processing_core SYSTEM ALSA)
endif()

target_link_libraries(
  nomadz_audio_processing_core FFTW3::fftw3 tensorflow::tensorflowlite
)

target_compile_options(
  nomadz_audio_processing_core PUBLIC -Wno-c++20-designator
)

# required since we are linking the components shared library to this
set_property(
  TARGET nomadz_audio_processing_core PROPERTY POSITION_INDEPENDENT_CODE ON
)

# components library

add_library(nomadz_audio_processing_components SHARED src/audio_processor.cpp)

target_include_directories(
  nomadz_audio_processing_components
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  nomadz_audio_processing_components
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  sensor_msgs
  nomadz_behavior_msgs
  nomadz_communication_msgs
  nomadz_audio_processing_msgs
  nomadz_configuration
  ament_index_cpp
)

target_link_libraries(
  nomadz_audio_processing_components nomadz_audio_processing_core
)

rclcpp_components_register_node(
  nomadz_audio_processing_components
  PLUGIN "nomadz_audio_processing::AudioProcessor"
  EXECUTABLE audio_processor
)

# installs

install(DIRECTORY include/ DESTINATION include/)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY ${MODEL_DOWNLOAD_DIR}/ DESTINATION share/${PROJECT_NAME}/models
)

install(
  TARGETS nomadz_audio_processing_components
  EXPORT nomadz_audio_processingTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()

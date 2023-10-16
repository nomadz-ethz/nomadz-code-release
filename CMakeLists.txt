cmake_minimum_required(VERSION 3.16.0)
cmake_policy(SET CMP0079 NEW)
cmake_policy(SET CMP0076 NEW)

set(SRC_DIR "${CMAKE_SOURCE_DIR}/Src")
set(UTIL_DIR "${CMAKE_SOURCE_DIR}/Util")

# Nomadz target and optional configuration

set(NOMADZ_TARGET
    "host"
    CACHE STRING "nomadz target mode")
option(ENABLE_TFLITE "Enable TensorFlow Lite" ON)
option(ENABLE_ROS "Enable ROS2" OFF)

set(NOMADZ_NG OFF)

if(NOMADZ_TARGET STREQUAL "ng" OR (NOT DEFINED NOMADZ_TARGET
                                   AND DEFINED CMAKE_TOOLCHAIN_FILE))
  message(STATUS "Compilation for next-generation NomadZ")
  set(NOMADZ_NG ON)
  set(ENABLE_ROS ON)
  if(DEFINED CMAKE_TOOLCHAIN_FILE)
    # Assume robot compilation with toolchain file
    set(NOMADZ_TARGET "v6")
  else()
    set(NOMADZ_TARGET "host")
  endif()
elseif(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_DISABLE_PRECOMPILE_HEADERS TRUE)
  if(DEFINED YOCTO_SDK)
    if(YOCTO_SDK STREQUAL "none" AND NOMADZ_TARGET STREQUAL "host")
      message(
        WARNING
          " YOCTO_SDK is set to none so cross compilation with bush will not work.\n"
          " To enable cross compilation re-run cmake with YOCTO_SDK correctly set."
      )
    else()
      if(NOT IS_ABSOLUTE "${YOCTO_SDK}")
        message(FATAL_ERROR "YOCTO_SDK must be an absolute path")
      endif()
      message(STATUS "Configuring v6 cross compilation toolchain")
      configure_file("${CMAKE_SOURCE_DIR}/CMake/v6-cross-64.cmake.in"
                      "${CMAKE_BINARY_DIR}/cross-config.cmake" @ONLY)
      set(CMAKE_TOOLCHAIN_FILE
          "${CMAKE_BINARY_DIR}/cross-config.cmake"
          CACHE PATH "toolchain file")
    endif()
  else()
    message(FATAL_ERROR "YOCTO_SDK not set, point it to yocto-sdk directory")
  endif()
endif()

# Note: project is called here because the CMAKE_TOOLCHAIN must be set before
# calling project
project(nomadz_legacy)

# Build type

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# C++ standard, CXX flags / features

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS_DEBUG "-ggdb")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_EXTENSIONS ON)

# CMake dependency resolution behavior for threads

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

# Global compile definitions

add_compile_definitions(LINUX)

if(CMAKE_CROSSCOMPILING)
  message(STATUS "Setting up v6 compilation")
  add_compile_definitions(BH_DIR="/home/nao/")
  add_compile_definitions(ROBOT_V6)
else()
  message(STATUS "Setting up host compilation")
  add_compile_definitions(BH_DIR="${CMAKE_SOURCE_DIR}")
  add_compile_definitions(YOCTO_SDK="${YOCTO_SDK}")
endif()

if(ENABLE_ROS)
  add_compile_definitions(ENABLE_ROS)
endif()

if(ENABLE_TFLITE)
  add_compile_definitions(ENABLE_TFLITE)
endif()

# Global compile options

add_compile_options(-mfpmath=sse -msse4)
# FIXME: force pthread option to be included everywhere for PCH to work
add_compile_options(-pthread)

# Dependency resolution

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake/")

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set(CMAKE_CXX_COMPILER_LAUNCHER ccache)
  set(CMAKE_C_COMPILER_LAUNCHER ccache)
endif(CCACHE_FOUND)

find_package(ALSA REQUIRED)

find_package(Eigen3 REQUIRED)
if(DEFINED EIGEN3_INCLUDE_DIR)
  list(APPEND Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(OpenCV REQUIRED)

find_package(
  Boost
  COMPONENTS filesystem system
  REQUIRED)

find_package(Threads COMPONENTS)

find_package(JPEG REQUIRED)

find_package(LAPACK REQUIRED)

if(NOMADZ_TARGET STREQUAL "host")
  find_package(Qt5 REQUIRED COMPONENTS Core Gui Svg OpenGL Help)
  set(Qt_LIBRARIES Qt5::Core Qt5::Gui Qt5::Svg Qt5::OpenGL Qt5::Help)
  find_package(OpenGL REQUIRED)
  find_package(LibXml2 REQUIRED)
  if(DEFINED LIBXML2_INCLUDE_DIR)
    list(APPEND LIBXML2_INCLUDE_DIRS ${LIBXML2_INCLUDE_DIR})
  endif()
  find_package(GLEW REQUIRED)
elseif(CMAKE_CROSSCOMPILING)
  find_package(Msgpack REQUIRED)
endif()

if(ENABLE_TFLITE)
  if(CMAKE_CROSSCOMPILING)
    # Check if tensorflow-lite is installed e.g. in SDK.
    find_library(TF_LITE_LIB tensorflow-lite)
    if(TF_LITE_LIB)
      message(STATUS "Using pre-installed tensorflow-lite: ${TF_LITE_LIB}")
      add_library(tensorflow-lite INTERFACE IMPORTED)
      target_link_libraries(tensorflow-lite INTERFACE ${TF_LITE_LIB})
    else()
      message(STATUS "Pre-installed tensorflow-lite not found")
    endif()
  endif()
endif()

# Configure (external) libraries and subprojects in Util
add_subdirectory(Util)

# Import ament CMake after Tensorflow (optionally compiled in Util) to prevent a
# target clash
if(ENABLE_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(rcutils REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(nomadz_msgs REQUIRED)
  ament_export_dependencies(nomadz_msgs)
endif()

# Global compile warnings flags
add_compile_options(
  -Wall
  -Wno-deprecated
  -Wno-address
  -Wno-strict-aliasing
  -Wno-reorder
  -Wno-sign-compare
  -Wno-switch
  -Wno-unused-label
  -Wno-mismatched-tags
  -Wno-init-self)

if(CMAKE_CROSSCOMPILING)
  add_compile_options(-Wno-attributes)
endif()

# Needed to speed up compilation significantly.
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION
                                              VERSION_GREATER_EQUAL 14)
  add_compile_options(-fpch-instantiate-templates)
endif()

# SPL GC and TC headers

add_subdirectory(Src/SPL)

# Core

add_subdirectory(Src/Core)
install(
  DIRECTORY "${CMAKE_SOURCE_DIR}/Src/Core"
  DESTINATION "include/nomadz_legacy"
  FILES_MATCHING
  PATTERN "*.h")

# Representations and tools 

# FIXME: representations is the only target that does
# not have its own subdirectory because Representations and Tools depend on each
# other so we need to do this in the top-level cmake file
add_library(representations OBJECT "")

# add source files to the representations target
add_subdirectory(Src/Representations)
add_subdirectory(Src/Tools)

if(CMAKE_CROSSCOMPILING)
  target_sources(representations PRIVATE "Src/Nao/NaoCamera.cpp"
                                         "Src/Nao/NaoCamera.h")
endif()

if(NOMADZ_TARGET STREQUAL "host")
  target_compile_definitions(representations PRIVATE TARGET_SIM)
else()
  target_compile_definitions(representations PRIVATE TARGET_ROBOT)
endif()

if(NOMADZ_NG OR NOMADZ_TARGET STREQUAL "host")
  set_property(TARGET representations PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()

target_include_directories(
  representations PUBLIC ${Eigen3_INCLUDE_DIRS} ${SRC_DIR}
                         ${SRC_DIR}/Apps/naobridge/include)
target_link_libraries(
  representations PUBLIC core gc_tc_headers snappy ${OpenCV_LIBS}
                         ${JPEG_LIBRARIES} Threads::Threads)

install(
  DIRECTORY "Representations"
  DESTINATION "include/nomadz_legacy"
  FILES_MATCHING
  PATTERN "*.h")

# Modules

add_subdirectory(Src/Modules)

# Host libraries and apps

if(NOMADZ_TARGET STREQUAL "host")
  # SimRobot
  add_subdirectory(Src/Apps/SimRobot)

  # Controller
  add_subdirectory(Src/Controller)

  # bush
  add_subdirectory(Src/Apps/bush)

  # URC
  add_subdirectory(Src/Apps/URC)

  # XBoxJoystick
  add_subdirectory(Src/Apps/xboxJoystick)

  # rf (random forests training tools)
  add_subdirectory(Src/Apps/rf)
endif()

# Robot libraries and apps

if(CMAKE_CROSSCOMPILING)
  # naobridge
  add_subdirectory(Src/Apps/naobridge)

  # Camera Register Tool
  add_subdirectory(Src/Apps/cameraRegTool)
endif()

# nomadz - simulation library or robot executable

set(NOMADZ_PROCESS_SRCS
    "Src/Processes/Cognition.cpp" "Src/Processes/Cognition.h"
    "Src/Processes/Debug.cpp" "Src/Processes/Debug.h"
    "Src/Processes/Motion.cpp" "Src/Processes/Motion.h")

if(NOMADZ_TARGET STREQUAL "host")
  add_library(nomadz SHARED ${NOMADZ_PROCESS_SRCS})

  target_compile_definitions(nomadz PRIVATE TARGET_SIM)
  target_link_libraries(nomadz PRIVATE Controller)

  set_target_properties(nomadz PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                                          "${CMAKE_BINARY_DIR}")

  add_custom_target(
    SimulatedNao ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink $<TARGET_FILE_NAME:nomadz>
            ${CMAKE_BINARY_DIR}/libSimulatedNao.so
    DEPENDS nomadz)
else() # v6
  if(NOMADZ_NG)
    add_library(nomadz SHARED ${NOMADZ_PROCESS_SRCS})
  else()
    add_executable(nomadz ${NOMADZ_PROCESS_SRCS} "Src/Apps/nomadz/Robot.cpp"
                          "Src/Apps/nomadz/Robot.h" "Src/Apps/nomadz/Main.cpp")
  endif()

  target_compile_definitions(nomadz PRIVATE TARGET_ROBOT)
  set_target_properties(nomadz PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                          "${CMAKE_BINARY_DIR}")
endif()

if(NOMADZ_NG)
  set_target_properties(nomadz PROPERTIES OUTPUT_NAME nomadz_legacy)
endif()

target_link_libraries(
  nomadz
  PRIVATE modules
          behavior_modules
          representations
          core
          ${Boost_LIBRARIES}
          ${ALSA_LIBRARIES}
          Threads::Threads
          ${CMAKE_DL_LIBS})

target_include_directories(
  nomadz PUBLIC $<BUILD_INTERFACE:${SRC_DIR}>
                $<INSTALL_INTERFACE:include/nomadz_legacy>)

if(ENABLE_ROS)
  install(
    TARGETS nomadz
    EXPORT nomadzTargets
    INCLUDES
    DESTINATION "include")
else()
  install(
    TARGETS nomadz
    INCLUDES
    DESTINATION "include")
endif()

# ament cmake calls (for ROS2 build) must be done in the top level CMakeLists
if(ENABLE_ROS)
  find_package(rclcpp REQUIRED)
  ament_target_dependencies(core PUBLIC rclcpp)
  ament_target_dependencies(nomadz PUBLIC rclcpp)

  # FIXME: Explicit dependency on Python should not be needed.
  if(NOMADZ_NG AND CMAKE_CROSSCOMPILING)
    find_package(Python3 REQUIRED)
    target_link_libraries(naobridge PUBLIC ${Python3_LIBRARY})
    target_link_libraries(nomadz PUBLIC ${Python3_LIBRARY})
  endif()

  find_package(nomadz_msgs REQUIRED)
  ament_export_dependencies(nomadz_msgs)
  ament_target_dependencies(core PUBLIC rclcpp nomadz_msgs)
  ament_target_dependencies(representations PUBLIC nomadz_msgs)
  ament_target_dependencies(modules PUBLIC nomadz_msgs)
  ament_target_dependencies(behavior_modules PUBLIC nomadz_msgs)
  ament_target_dependencies(nomadz PUBLIC rclcpp nomadz_msgs)
  find_package(sensor_msgs REQUIRED)
  ament_export_dependencies(sensor_msgs)
  ament_target_dependencies(representations PUBLIC sensor_msgs)
  ament_target_dependencies(modules PUBLIC sensor_msgs)

  ament_export_targets(nomadzTargets HAS_LIBRARY_TARGET)

  if(NOMADZ_TARGET STREQUAL "host")
    ament_target_dependencies(Controller PUBLIC nomadz_msgs)
    ament_target_dependencies(SimRobotCore2 PUBLIC nomadz_msgs)
    ament_target_dependencies(rf PUBLIC nomadz_msgs)
    ament_target_dependencies(URC PUBLIC nomadz_msgs)
    ament_target_dependencies(bush PUBLIC nomadz_msgs)
    ament_target_dependencies(SimRobot PUBLIC rclcpp nomadz_msgs)

    target_compile_definitions(SimRobot PRIVATE ENABLE_ROS)
    target_compile_definitions(SimRobot PRIVATE ENABLE_ROS_TARGET)

    target_compile_definitions(bush PRIVATE ENABLE_ROS_TARGET)
  endif()

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()
endif()

# Targets to run formatting and linting with build system
file(
  GLOB_RECURSE
  FORMAT_CXX_SRC_FILES
  ${SRC_DIR}/Modules/*.[tch]pp
  ${SRC_DIR}/Modules/*.h
  ${SRC_DIR}/Representations/*.[tch]pp
  ${SRC_DIR}/Representations/*.h
  ${SRC_DIR}/Core/*.[tch]pp
  ${SRC_DIR}/Core/*.h
  ${SRC_DIR}/Tools/*.[tch]pp
  ${SRC_DIR}/Tools/*.h
  ${SRC_DIR}/Nao/*.[tch]pp
  ${SRC_DIR}/Nao/*.h
  ${SRC_DIR}/Processes/*.[tch]pp
  ${SRC_DIR}/Processes/*.h
  ${SRC_DIR}/Apps/nomadz/*.h
  ${SRC_DIR}/Apps/nomadz/*.[tch]pp
  ${SRC_DIR}/Apps/naobridge/*.h
  ${SRC_DIR}/Apps/naobridge/*.[tch]pp
  ${SRC_DIR}/Apps/xboxJoystick/*.h
  ${SRC_DIR}/Apps/xboxJoystick/*.[tch]pp
  ${SRC_DIR}/Apps/SimRobot/Src/*.h
  ${SRC_DIR}/Apps/SimRobot/Src/*.[tch]pp)

include("CMake/clang-format.cmake")
include("CMake/clang-tidy.cmake")

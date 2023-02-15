cmake_minimum_required(VERSION 3.16.0)
cmake_policy(SET CMP0079 NEW)
cmake_policy(SET CMP0076 NEW)

set(SRC_DIR "${CMAKE_SOURCE_DIR}/Src")
set(UTIL_DIR "${CMAKE_SOURCE_DIR}/Util")

# CONFIG ###

set(NOMADZ_TARGET
    "host-64"
    CACHE STRING "nomadz target mode")
if(NOMADZ_TARGET STREQUAL "v6")
  set(NOMADZ_TARGET "v6-64")
endif()

set(ENABLE_TFLITE
  ON
  CACHE STRING "Enable Tensorflow Lite")
set(ENABLE_ROS
  OFF
  CACHE STRING "Enable ROS2")

set(NOMADZ_NG OFF)
if(NOMADZ_TARGET STREQUAL "ng" OR (NOT DEFINED NOMADZ_TARGET AND DEFINED CMAKE_TOOLCHAIN_FILE))
  message(STATUS "Compilation for next-generation NomadZ")
  set(NOMADZ_NG ON)
  set(ENABLE_ROS ON)
  if(DEFINED CMAKE_TOOLCHAIN_FILE)
    # Assume robot compilation with toolchain file
    set(NOMADZ_TARGET "v6-64")
    add_compile_definitions(BH_DIR="/home/nao/")
  else()
    set(NOMADZ_TARGET "host-64")
    add_compile_definitions(BH_DIR="${CMAKE_SOURCE_DIR}")
  endif()
elseif(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_DISABLE_PRECOMPILE_HEADERS TRUE)
  if(NOMADZ_TARGET STREQUAL "host"
     OR NOMADZ_TARGET STREQUAL "host-32"
     OR NOMADZ_TARGET STREQUAL "host-64"
     OR NOMADZ_TARGET STREQUAL "v6-32"
     OR NOMADZ_TARGET STREQUAL "v6-64")
    if(DEFINED YOCTO_SDK)
      if(YOCTO_SDK STREQUAL "none" AND 
          (NOMADZ_TARGET STREQUAL "host" 
           OR NOMADZ_TARGET STREQUAL "host-64" 
           OR NOMADZ_TARGET STREQUAL "host-32"))
        message(WARNING 
          " YOCTO_SDK is set to none so cross compilation with bush will not work.\n"
          " To enable cross compilation re-run cmake with YOCTO_SDK correctly set.")
      elseif(NOT IS_ABSOLUTE "${YOCTO_SDK}"
         OR (((NOMADZ_TARGET STREQUAL "v6-32" OR NOMADZ_TARGET STREQUAL "host-32")
              AND NOT
                  EXISTS
                  "${YOCTO_SDK}/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-gcc"
             )
             OR ((NOMADZ_TARGET STREQUAL "v6-64"
                  OR NOMADZ_TARGET STREQUAL "host"
                  OR NOMADZ_TARGET STREQUAL "host-64")
                 AND NOT
                     EXISTS
                     "${YOCTO_SDK}/sysroots/x86_64-pokysdk-linux/usr/bin/x86_64-poky-linux/x86_64-poky-linux-gcc"
                )
            ))
        message(
          FATAL_ERROR
            "YOCTO_SDK does not point to the correct location or is not an absolute path"
        )
      endif()
      if(NOMADZ_TARGET STREQUAL "v6-32")
        configure_file("${CMAKE_SOURCE_DIR}/CMake/v6-cross-32.cmake.in"
                       "${CMAKE_BINARY_DIR}/cross-config.cmake" @ONLY)
      elseif(NOMADZ_TARGET STREQUAL "v6-64")
        configure_file("${CMAKE_SOURCE_DIR}/CMake/v6-cross-64.cmake.in"
                       "${CMAKE_BINARY_DIR}/cross-config.cmake" @ONLY)
      endif()
    else()
      message(FATAL_ERROR "YOCTO_SDK not set, point it to yocto-sdk directory")
    endif()
  endif()
endif()

# GLOBAL COMPILE DEFINITIONS ###

if(NOMADZ_TARGET STREQUAL "host" OR NOMADZ_TARGET STREQUAL "host-64")
  set(NOMADZ_TARGET "host")
  add_definitions(-DTARGET_64BIT)
endif()
if(NOMADZ_TARGET STREQUAL "host-32")
  message(STATUS "Build legacy 32-bit Nao binaries from root tools")
  set(NOMADZ_TARGET "host")
endif()

if(NOMADZ_TARGET STREQUAL "v6-32")
  message(STATUS "Setting up 32-bit v6 compilation")
  set(CMAKE_TOOLCHAIN_FILE
      "${CMAKE_BINARY_DIR}/cross-config.cmake"
      CACHE PATH "toolchain file")
  add_definitions(-DROBOT_V6)
elseif(NOMADZ_TARGET STREQUAL "v6-64")
  message(STATUS "Setting up 64-bit v6 compilation")
  set(CMAKE_TOOLCHAIN_FILE
      "${CMAKE_BINARY_DIR}/cross-config.cmake"
      CACHE PATH "toolchain file")
  add_definitions(-DROBOT_V6)
elseif(NOMADZ_TARGET STREQUAL "host")
  message(STATUS "Setting up host compilation")
  add_definitions(-DYOCTO_SDK="${YOCTO_SDK}")
elseif(NOMADZ_TARGET STREQUAL "host-only")
  message(STATUS "Setting up host only compilation")
elseif(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  message(
    FATAL_ERROR
      "NOMADZ_TARGET not 'host', 'host-64', 'host-only', 'v6-32' ('v6') or 'v6-64'")
endif()

add_definitions(-DLINUX)

# PROJECT ###

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

project(nomadz_legacy)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# CXX flags

set(CMAKE_CXX_FLAGS_DEBUG "-ggdb")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

if(ENABLE_ROS)
  set(CMAKE_CXX_STANDARD 17)
  add_compile_definitions(ENABLE_ROS)
else()
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_EXTENSIONS ON)

# Enable SSE
add_compile_options(-mfpmath=sse -msse4)

# FIXME: set special paths for ubuntu14 setups for the moment
find_program(LSB_RELEASE_EXEC lsb_release)
if((NOMADZ_TARGET STREQUAL "host" OR NOMADZ_TARGET STREQUAL "host-only")
   AND LSB_RELEASE_EXEC)
  execute_process(
    COMMAND ${LSB_RELEASE_EXEC} -is
    OUTPUT_VARIABLE LSB_RELEASE_ID
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(
    COMMAND ${LSB_RELEASE_EXEC} -rs
    OUTPUT_VARIABLE LSB_RELEASE_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(LSB_RELEASE_ID STREQUAL "Ubuntu"
     AND (LSB_RELEASE_VERSION STREQUAL "14.04" OR LSB_RELEASE_VERSION STREQUAL
                                                  "16.04"))
    message(
      WARNING "Applying special configuration for Ubuntu ${LSB_RELEASE_VERSION}"
    )
    list(APPEND CMAKE_PREFIX_PATH "${UTIL_DIR}/opencv3/ubuntu14")
    set(ALSA_LIBRARIES "/usr/lib/x86_64-linux-gnu/libasound.so.2")
    set(ALSA_INCLUDE_DIRS "/usr/include")
    set(LIBGFORTRAN_LIBRARIES "/usr/lib/x86_64-linux-gnu/libgfortran.so.3")
  else()
    find_package(ALSA REQUIRED)
  endif()
else()
  find_package(ALSA REQUIRED)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake/")

find_package(Eigen3 REQUIRED)
if(DEFINED EIGEN3_INCLUDE_DIR)
  list(APPEND Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
find_package(OpenCV REQUIRED)
find_package(
  Boost
  COMPONENTS filesystem system
  REQUIRED)
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads COMPONENTS)
set(THREAD_LIBS Threads::Threads)
# FIXME: force pthread option to be included everywhere for PCH to work
add_compile_options(-pthread)

if(ENABLE_TFLITE)
  add_compile_definitions(ENABLE_TFLITE)
  set(ABSL_PROPAGATE_CXX_STD ON)
endif()

find_package(JPEG REQUIRED)

if(NOMADZ_TARGET STREQUAL "host" OR NOMADZ_TARGET STREQUAL "host-only")
  find_package(LAPACK REQUIRED)
  find_package(Qt5 COMPONENTS Core Gui Svg OpenGL Help)
  if(Qt5_FOUND)
    set(Qt_LIBRARIES Qt5::Core Qt5::Gui Qt5::Svg Qt5::OpenGL Qt5::Help)
  else()
    message("Qt5 not found - falling back to Qt4")
    find_package(
      Qt4
      COMPONENTS QtCore QtGui QtSvg QtOpenGL QtHelp
      REQUIRED)
    set(Qt_LIBRARIES Qt4::QtCore Qt4::QtGui Qt4::QtSvg Qt4::QtOpenGL
                     Qt4::QtHelp)
  endif()
  find_package(OpenGL REQUIRED)
  find_package(LibXml2 REQUIRED)
  if(DEFINED LIBXML2_INCLUDE_DIR)
    list(APPEND LIBXML2_INCLUDE_DIRS ${LIBXML2_INCLUDE_DIR})
  endif()
  find_package(GLEW REQUIRED)
elseif(NOMADZ_TARGET STREQUAL "v6-32" OR NOMADZ_TARGET STREQUAL "v6-64")
  find_package(Msgpack REQUIRED)
  if(NOMADZ_TARGET STREQUAL "v6-64")
    find_package(LAPACK REQUIRED)
    find_library(LIBGFORTRAN_LIBRARIES NAMES libgfortran.so)
  endif()
endif()

# This will create targets for all the external libraries in Util
add_subdirectory(${UTIL_DIR})

# Import ament CMake after Tensorflow to prevent a target clash
if(ENABLE_ROS)
  find_package(ament_cmake REQUIRED)

  ament_export_targets(nomadzTargets HAS_LIBRARY_TARGET)
endif()

# After configuring all the external dependencies, we add compile warning
# options
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
  -Wno-init-self
)
# Needed to speed up compilation significantly.
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 14)
  add_compile_options(
    -fpch-instantiate-templates
  )
endif()

if(NOMADZ_TARGET STREQUAL "v6-32" OR NOMADZ_TARGET STREQUAL "v6-64")
  add_compile_options(-Wno-attributes)
endif()

# ##############################################################################
# SPL COMMON ###
# ##############################################################################

# standard RoboCup headers
add_subdirectory(Src/SPL)
include_directories(${GAME_CONTROLLER_INCLUDE_DIRS})

# ##############################################################################
# Core ###
# ##############################################################################

add_library(core OBJECT "")
add_subdirectory(Src/Core)
install(DIRECTORY "${CMAKE_SOURCE_DIR}/Src/Core"
  DESTINATION "include/nomadz_legacy"
  FILES_MATCHING
  PATTERN "*.h"
)

# ##############################################################################
# Representations and Tools ###
# ##############################################################################

# FIXME(albanesg): the target is configured here because the source files are
# split between the two directories
add_library(representations OBJECT "")

if(NOMADZ_TARGET STREQUAL "v6-32" OR NOMADZ_TARGET STREQUAL "v6-64")
  target_sources(representations PRIVATE "Src/Nao/NaoCamera.cpp"
                                         "Src/Nao/NaoCamera.h")
endif()

# FIXME(albanesg): these only add source files to representations
add_subdirectory(Src/Representations)
add_subdirectory(Src/Tools)

if(NOMADZ_TARGET STREQUAL "host" OR NOMADZ_TARGET STREQUAL "host-only")
  target_compile_definitions(representations PRIVATE TARGET_SIM)
else()
  target_compile_definitions(representations PRIVATE TARGET_ROBOT)
endif()

if(NOMADZ_NG OR NOMADZ_TARGET STREQUAL "host" OR NOMADZ_TARGET STREQUAL "host-only")
  set_property(TARGET representations PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()

target_include_directories(
  representations PUBLIC ${Eigen3_INCLUDE_DIRS} ${SRC_DIR}
                         ${SRC_DIR}/Apps/naobridge/include)
target_link_libraries(representations PUBLIC core snappy ${OpenCV_LIBS}
                                             ${JPEG_LIBRARIES} ${THREAD_LIBS})

install(DIRECTORY "${CMAKE_SOURCE_DIR}/Src/Representations"
  DESTINATION "include/nomadz_legacy"
  FILES_MATCHING
  PATTERN "*.h"
)

# ##############################################################################
# Modules ###
# ##############################################################################

add_library(modules OBJECT "")
add_library(behavior_modules OBJECT "")
add_subdirectory(Src/Modules)

# ##############################################################################
# bush ###
# ##############################################################################

if(NOMADZ_TARGET STREQUAL "host" OR NOMADZ_TARGET STREQUAL "host-only")
  add_executable(bush)
  add_subdirectory(Src/Apps/bush)
  install(TARGETS bush
    RUNTIME DESTINATION "bin")
endif()

# ##############################################################################
# SimRobot ###
# ##############################################################################

if(NOMADZ_TARGET STREQUAL "host" OR NOMADZ_TARGET STREQUAL "host-only")
  add_library(SimRobotCore2 SHARED)
  add_library(SimRobotEditor SHARED)
  add_library(SimRobotHelp SHARED)
  add_executable(SimRobot)
  add_subdirectory(Src/Apps/SimRobot)
  install(TARGETS SimRobot SimRobotCore2 SimRobotEditor SimRobotHelp
          RUNTIME DESTINATION "bin"
          LIBRARY DESTINATION "lib")
endif()

# ##############################################################################
# URC ###
# ##############################################################################

if(NOMADZ_TARGET STREQUAL "host" OR NOMADZ_TARGET STREQUAL "host-only")
  add_executable(URC)
  add_subdirectory(Src/Apps/URC)
endif()

# ##############################################################################
# Controller ###
# ##############################################################################

if(NOMADZ_TARGET STREQUAL "host" OR NOMADZ_TARGET STREQUAL "host-only")
  add_library(Controller STATIC "")
  add_subdirectory(Src/Controller)
endif()

# ##############################################################################
# naobridge ###
# ##############################################################################

if(NOMADZ_TARGET STREQUAL "v6-32" OR NOMADZ_TARGET STREQUAL "v6-64")
  add_executable(naobridge)
  add_subdirectory(Src/Apps/naobridge)
endif()

# ##############################################################################
# nomadz ###
# ##############################################################################

set(NOMADZ_PROCESS_SRCS
  "Src/Processes/Cognition.cpp" "Src/Processes/Cognition.h"
  "Src/Processes/Debug.cpp" "Src/Processes/Debug.h"
  "Src/Processes/Motion.cpp" "Src/Processes/Motion.h"
)

if(NOMADZ_TARGET STREQUAL "host" OR NOMADZ_TARGET STREQUAL "host-only")
  add_library(
    nomadz SHARED ${NOMADZ_PROCESS_SRCS})

  target_compile_definitions(nomadz PRIVATE TARGET_SIM)
  target_link_libraries(
    nomadz
    PRIVATE Controller)

  add_custom_target(
    SimulatedNao ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink $<TARGET_FILE_NAME:nomadz>
            libSimulatedNao.so
    DEPENDS nomadz)
else() # v6-32, v6-64
  if(NOMADZ_NG)
    add_library(
      nomadz SHARED
      ${NOMADZ_PROCESS_SRCS})
  else()
    add_executable(
      nomadz
      ${NOMADZ_PROCESS_SRCS}
      "Src/Apps/nomadz/Robot.cpp"
      "Src/Apps/nomadz/Robot.h"
      "Src/Apps/nomadz/Main.cpp")
  endif()

  target_compile_definitions(nomadz PRIVATE TARGET_ROBOT)
endif()

if(NOMADZ_NG)
  set_target_properties(nomadz PROPERTIES OUTPUT_NAME nomadz_legacy)
endif()

target_link_libraries(
  nomadz
  PRIVATE
  modules
  behavior_modules
  representations
  core
  ${Boost_LIBRARIES}
  ${ALSA_LIBRARIES}
  ${THREAD_LIBS}
  ${LIBGFORTRAN_LIBRARIES}
  ${CMAKE_DL_LIBS})

target_include_directories(nomadz PUBLIC
  $<BUILD_INTERFACE:${SRC_DIR}>
  $<INSTALL_INTERFACE:include/nomadz_legacy>)

if(ENABLE_ROS)
  install(TARGETS nomadz
          EXPORT nomadzTargets
          INCLUDES DESTINATION "include")
else()
  install(TARGETS nomadz
    INCLUDES DESTINATION "include")
endif()

# ##############################################################################
# xboxJoystick ###
# ##############################################################################

if(NOMADZ_TARGET STREQUAL "host" OR NOMADZ_TARGET STREQUAL "host-only")
  add_executable(xboxJoystick "")
  target_sources(
    xboxJoystick
    PRIVATE ${SRC_DIR}/Apps/xboxJoystick/xboxJoystick.cpp
            ${SRC_DIR}/Modules/MotionControl/JoystickController/XBoxJoystick.cpp
            ${SRC_DIR}/Modules/MotionControl/JoystickController/XBoxJoystick.h)

  target_include_directories(
    xboxJoystick PRIVATE ${SRC_DIR}/Modules/MotionControl/JoystickController)
endif()

# ##############################################################################
# ROS2 ###
# ##############################################################################

if(ENABLE_ROS)
  find_package(rcutils REQUIRED)
  find_package(rclcpp REQUIRED)
  ament_target_dependencies(core PUBLIC rclcpp)
  ament_target_dependencies(nomadz PUBLIC rclcpp)

  find_package(nomadz_msgs REQUIRED)
  ament_export_dependencies(nomadz_msgs)
  ament_target_dependencies(core PUBLIC rclcpp nomadz_msgs)
  ament_target_dependencies(representations PUBLIC nomadz_msgs)
  ament_target_dependencies(modules PUBLIC nomadz_msgs)
  ament_target_dependencies(behavior_modules PUBLIC nomadz_msgs)
  ament_target_dependencies(nomadz PUBLIC nomadz_msgs)

  if(NOMADZ_TARGET STREQUAL "host" OR NOMADZ_TARGET STREQUAL "host-only")
    ament_target_dependencies(Controller PUBLIC nomadz_msgs)
    ament_target_dependencies(SimRobotCore2 PUBLIC nomadz_msgs)
    ament_target_dependencies(URC PUBLIC nomadz_msgs)
    ament_target_dependencies(bush PUBLIC nomadz_msgs)
    ament_target_dependencies(SimRobot PUBLIC rclcpp nomadz_msgs)

    target_compile_definitions(SimRobot PRIVATE ENABLE_ROS)
    target_compile_definitions(SimRobot PRIVATE ENABLE_ROS_TARGET)

    target_compile_definitions(bush PRIVATE ENABLE_ROS_TARGET)
  endif()
endif()

# ##############################################################################
# FORMAT AND LINT ###
# ##############################################################################

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
  ${SRC_DIR}/Apps/SimRobot/Src/*.[tch]pp
)

include("CMake/clang-format.cmake")
include("CMake/clang-tidy.cmake")

if(ENABLE_ROS)
  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()
endif()

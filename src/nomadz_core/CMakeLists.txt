cmake_minimum_required(VERSION 3.16)
project(nomadz_core)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(
  nomadz_core STATIC
  src/geometry/ros_conversion.cpp
  src/geometry/line.cpp
  src/math/distributions.cpp
  src/math/least_squares.cpp
  src/algorithms/lp_filter.cpp
)
set_property(TARGET nomadz_core PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(
  nomadz_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nomadz_core geometry_msgs)
target_link_libraries(nomadz_core Eigen3::Eigen)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS nomadz_core
  EXPORT nomadz_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    test_core
    test/test_clamp.cpp
    test/test_projection.cpp
    test/test_polynom.cpp
    test/test_interpolation.cpp
  )
  target_link_libraries(test_core nomadz_core Eigen3::Eigen)
endif()

ament_export_targets(nomadz_coreTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 geometry_msgs)

ament_package()

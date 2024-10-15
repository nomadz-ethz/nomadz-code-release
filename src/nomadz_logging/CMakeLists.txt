cmake_minimum_required(VERSION 3.16)
project(nomadz_logging)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS src/ProfilingNode.py DESTINATION lib/${PROJECT_NAME})

ament_package()

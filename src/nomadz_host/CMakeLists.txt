cmake_minimum_required(VERSION 3.16)

project(nomadz_host)

find_package(ament_cmake REQUIRED)
find_package(nao_camera CONFIG REQUIRED)
find_package(nao_lola_client REQUIRED)
find_package(example_nao_lola_control REQUIRED)
find_package(nomadz_kinematics REQUIRED)

ament_package()

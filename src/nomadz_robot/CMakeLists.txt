cmake_minimum_required(VERSION 3.16)

project(nomadz_robot)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nao_camera REQUIRED)
find_package(nao_lola_client REQUIRED)

ament_package()

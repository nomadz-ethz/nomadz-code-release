cmake_minimum_required(VERSION 3.16)

project(nomadz_bringup)

find_package(ament_cmake REQUIRED)
find_package(nao_camera REQUIRED)
find_package(nao_lola_client REQUIRED)
find_package(nomadz_proprioception REQUIRED)
find_package(nomadz_audio_processing REQUIRED)

if(CMAKE_CROSSCOMPILING)
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "sim_*" EXCLUDE
  )
else()
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()

ament_package()

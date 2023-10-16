add_executable(
  naobridge
  src/lola_bridge.cpp
  src/lola_bridge.h
  src/game_control.cpp
  src/game_control.h
  src/lola_frame.cpp
  src/lola_frame.h
  src/stl_ext.h)

target_include_directories(
  naobridge
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE src)

target_link_libraries(naobridge PRIVATE core rt ${CMAKE_DL_LIBS}
                                        ${Boost_LIBRARIES} ${Msgpack_LIBRARIES})

set_target_properties(naobridge PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                           "${CMAKE_BINARY_DIR}")

install(TARGETS naobridge)

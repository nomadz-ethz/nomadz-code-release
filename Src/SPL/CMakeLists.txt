# header only library target containing GameController and TeamComm headers
add_library(gc_tc_headers INTERFACE)
target_include_directories(gc_tc_headers
                           INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
  DESTINATION "include"
  FILES_MATCHING
  PATTERN "*.h")

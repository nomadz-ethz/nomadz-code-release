target_sources(URC PRIVATE UniversalResourceCompiler.cpp MofCompiler.cpp)

target_include_directories(URC PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_compile_definitions(URC PRIVATE TARGET_TOOL)

target_link_libraries(URC PRIVATE representations core)

add_custom_target(
  URC_run ALL
  COMMAND URC
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMENT "Run generated URC"
  SOURCES ${URC_SRC_FILES})

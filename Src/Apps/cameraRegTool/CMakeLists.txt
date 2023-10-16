add_executable(cameraRegTool "${SRC_DIR}/Apps/cameraRegTool/reg.cpp")
set_target_properties(cameraRegTool PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                               "${CMAKE_BINARY_DIR}")

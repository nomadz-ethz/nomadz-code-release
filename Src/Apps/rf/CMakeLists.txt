add_executable(
  rf
  "util/common.h"
  "command/validate.h"
  "command/train.cpp"
  "command/size.cpp"
  "command/copy.h"
  "command/size.h"
  "command/predict.h"
  "command/color.cpp"
  "command/all.h"
  "command/copy.cpp"
  "command/convert.cpp"
  "command/validate.cpp"
  "command/command.h"
  "command/color.h"
  "command/train.h"
  "command/predict.cpp"
  "command/convert.h"
  "rf.cpp"
  "rf.h")

target_include_directories(rf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  target_compile_definitions(rf PRIVATE MULTI)
  target_compile_options(rf PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(rf PRIVATE ${OpenMP_CXX_LIBRARIES})
endif()

target_compile_definitions(rf PRIVATE TARGET_TOOL)
target_link_libraries(rf PRIVATE core representations ${OpenCV_LIBS})

set_target_properties(rf PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                    "${CMAKE_BINARY_DIR}")

add_executable(
  xboxJoystick
  xboxJoystick ${SRC_DIR}/Apps/xboxJoystick/xboxJoystick.cpp
  ${SRC_DIR}/Modules/MotionControl/JoystickController/XBoxJoystick.cpp
  ${SRC_DIR}/Modules/MotionControl/JoystickController/XBoxJoystick.h)

target_include_directories(
  xboxJoystick PRIVATE ${SRC_DIR}/Modules/MotionControl/JoystickController)

set_target_properties(xboxJoystick PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                              ${CMAKE_BINARY_DIR})

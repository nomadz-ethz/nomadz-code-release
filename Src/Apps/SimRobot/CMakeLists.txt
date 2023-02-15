
target_sources(
  SimRobotCore2
  PRIVATE "Src/SimRobotCore2/ActuatorsWidget.cpp"
          "Src/SimRobotCore2/ActuatorsWidget.h"
          "Src/SimRobotCore2/CoreModule.cpp"
          "Src/SimRobotCore2/CoreModule.h"
          "Src/SimRobotCore2/Parser/Element.cpp"
          "Src/SimRobotCore2/Parser/Element.h"
          "Src/SimRobotCore2/Parser/Parser.cpp"
          "Src/SimRobotCore2/Parser/Parser.h"
          "Src/SimRobotCore2/Parser/Reader.cpp"
          "Src/SimRobotCore2/Parser/Reader.h"
          "Src/SimRobotCore2/Platform/Assert.cpp"
          "Src/SimRobotCore2/Platform/Assert.h"
          "Src/SimRobotCore2/Platform/OffscreenRenderer.cpp"
          "Src/SimRobotCore2/Platform/OffscreenRenderer.h"
          "Src/SimRobotCore2/Platform/OpenGL.h"
          "Src/SimRobotCore2/Platform/System.cpp"
          "Src/SimRobotCore2/Platform/System.h"
          "Src/SimRobotCore2/SensorWidget.cpp"
          "Src/SimRobotCore2/SensorWidget.h"
          "Src/SimRobotCore2/SimObjectRenderer.cpp"
          "Src/SimRobotCore2/SimObjectRenderer.h"
          "Src/SimRobotCore2/SimObjectWidget.cpp"
          "Src/SimRobotCore2/SimObjectWidget.h"
          "Src/SimRobotCore2/SimRobotCore2.h"
          "Src/SimRobotCore2/SimRobotCore2.qrc"
          "Src/SimRobotCore2/Simulation/Actuators/Actuator.cpp"
          "Src/SimRobotCore2/Simulation/Actuators/Actuator.h"
          "Src/SimRobotCore2/Simulation/Actuators/Hinge.cpp"
          "Src/SimRobotCore2/Simulation/Actuators/Hinge.h"
          "Src/SimRobotCore2/Simulation/Actuators/Joint.cpp"
          "Src/SimRobotCore2/Simulation/Actuators/Joint.h"
          "Src/SimRobotCore2/Simulation/Actuators/Slider.cpp"
          "Src/SimRobotCore2/Simulation/Actuators/Slider.h"
          "Src/SimRobotCore2/Simulation/Appearances/Appearance.cpp"
          "Src/SimRobotCore2/Simulation/Appearances/Appearance.h"
          "Src/SimRobotCore2/Simulation/Appearances/BoxAppearance.cpp"
          "Src/SimRobotCore2/Simulation/Appearances/BoxAppearance.h"
          "Src/SimRobotCore2/Simulation/Appearances/CapsuleAppearance.cpp"
          "Src/SimRobotCore2/Simulation/Appearances/CapsuleAppearance.h"
          "Src/SimRobotCore2/Simulation/Appearances/ComplexAppearance.cpp"
          "Src/SimRobotCore2/Simulation/Appearances/ComplexAppearance.h"
          "Src/SimRobotCore2/Simulation/Appearances/CylinderAppearance.cpp"
          "Src/SimRobotCore2/Simulation/Appearances/CylinderAppearance.h"
          "Src/SimRobotCore2/Simulation/Appearances/SphereAppearance.cpp"
          "Src/SimRobotCore2/Simulation/Appearances/SphereAppearance.h"
          "Src/SimRobotCore2/Simulation/Axis.cpp"
          "Src/SimRobotCore2/Simulation/Axis.h"
          "Src/SimRobotCore2/Simulation/Body.cpp"
          "Src/SimRobotCore2/Simulation/Body.h"
          "Src/SimRobotCore2/Simulation/Compound.cpp"
          "Src/SimRobotCore2/Simulation/Compound.h"
          "Src/SimRobotCore2/Simulation/Geometries/BoxGeometry.cpp"
          "Src/SimRobotCore2/Simulation/Geometries/BoxGeometry.h"
          "Src/SimRobotCore2/Simulation/Geometries/CapsuleGeometry.cpp"
          "Src/SimRobotCore2/Simulation/Geometries/CapsuleGeometry.h"
          "Src/SimRobotCore2/Simulation/Geometries/CylinderGeometry.cpp"
          "Src/SimRobotCore2/Simulation/Geometries/CylinderGeometry.h"
          "Src/SimRobotCore2/Simulation/Geometries/Geometry.cpp"
          "Src/SimRobotCore2/Simulation/Geometries/Geometry.h"
          "Src/SimRobotCore2/Simulation/Geometries/SphereGeometry.cpp"
          "Src/SimRobotCore2/Simulation/Geometries/SphereGeometry.h"
          "Src/SimRobotCore2/Simulation/GraphicalObject.cpp"
          "Src/SimRobotCore2/Simulation/GraphicalObject.h"
          "Src/SimRobotCore2/Simulation/Masses/BoxMass.cpp"
          "Src/SimRobotCore2/Simulation/Masses/BoxMass.h"
          "Src/SimRobotCore2/Simulation/Masses/InertiaMatrixMass.cpp"
          "Src/SimRobotCore2/Simulation/Masses/InertiaMatrixMass.h"
          "Src/SimRobotCore2/Simulation/Masses/Mass.cpp"
          "Src/SimRobotCore2/Simulation/Masses/Mass.h"
          "Src/SimRobotCore2/Simulation/Masses/SphereMass.cpp"
          "Src/SimRobotCore2/Simulation/Masses/SphereMass.h"
          "Src/SimRobotCore2/Simulation/Motors/Motor.h"
          "Src/SimRobotCore2/Simulation/Motors/ServoMotor.cpp"
          "Src/SimRobotCore2/Simulation/Motors/ServoMotor.h"
          "Src/SimRobotCore2/Simulation/Motors/VelocityMotor.cpp"
          "Src/SimRobotCore2/Simulation/Motors/VelocityMotor.h"
          "Src/SimRobotCore2/Simulation/PhysicalObject.cpp"
          "Src/SimRobotCore2/Simulation/PhysicalObject.h"
          "Src/SimRobotCore2/Simulation/Scene.cpp"
          "Src/SimRobotCore2/Simulation/Scene.h"
          "Src/SimRobotCore2/Simulation/Sensors/Accelerometer.cpp"
          "Src/SimRobotCore2/Simulation/Sensors/Accelerometer.h"
          "Src/SimRobotCore2/Simulation/Sensors/ApproxDistanceSensor.cpp"
          "Src/SimRobotCore2/Simulation/Sensors/ApproxDistanceSensor.h"
          "Src/SimRobotCore2/Simulation/Sensors/Camera.cpp"
          "Src/SimRobotCore2/Simulation/Sensors/Camera.h"
          "Src/SimRobotCore2/Simulation/Sensors/CollisionSensor.cpp"
          "Src/SimRobotCore2/Simulation/Sensors/CollisionSensor.h"
          "Src/SimRobotCore2/Simulation/Sensors/DepthImageSensor.cpp"
          "Src/SimRobotCore2/Simulation/Sensors/DepthImageSensor.h"
          "Src/SimRobotCore2/Simulation/Sensors/Gyroscope.cpp"
          "Src/SimRobotCore2/Simulation/Sensors/Gyroscope.h"
          "Src/SimRobotCore2/Simulation/Sensors/Sensor.cpp"
          "Src/SimRobotCore2/Simulation/Sensors/Sensor.h"
          "Src/SimRobotCore2/Simulation/Sensors/SingleDistanceSensor.cpp"
          "Src/SimRobotCore2/Simulation/Sensors/SingleDistanceSensor.h"
          "Src/SimRobotCore2/Simulation/SimObject.cpp"
          "Src/SimRobotCore2/Simulation/SimObject.h"
          "Src/SimRobotCore2/Simulation/Simulation.cpp"
          "Src/SimRobotCore2/Simulation/Simulation.h"
          "Src/SimRobotCore2/Tools/ODETools.h"
          "Src/SimRobotCore2/Tools/OpenGLTools.cpp"
          "Src/SimRobotCore2/Tools/OpenGLTools.h"
          "Src/SimRobotCore2/Tools/Pose3.h"
          "Src/SimRobotCore2/Tools/Texture.cpp"
          "Src/SimRobotCore2/Tools/Texture.h")

target_compile_definitions(SimRobotCore2 PRIVATE TARGET_TOOL)
target_compile_definitions(SimRobotCore2 PUBLIC dSINGLE)
target_include_directories(
  SimRobotCore2
  PRIVATE ${LIBXML2_INCLUDE_DIRS}
          ${CMAKE_CURRENT_SOURCE_DIR}/Util/ode/Linux/include
          ${CMAKE_CURRENT_SOURCE_DIR}/Src/SimRobotCore2)
set_target_properties(SimRobotCore2 PROPERTIES AUTOMOC ON AUTORCC ON)
target_link_libraries(
  SimRobotCore2
  PRIVATE core)
target_link_libraries(SimRobotCore2 PUBLIC
         ${Qt_LIBRARIES} ${LIBXML2_LIBRARIES} ${OPENGL_LIBRARIES}
         ${GLEW_LIBRARIES}
         ${CMAKE_CURRENT_SOURCE_DIR}/Util/ode/Linux64/lib/libode.a)

# ##############################################################################
# SimRobotEditor ###
# ##############################################################################

target_sources(
  SimRobotEditor
  PRIVATE "Src/SimRobotEditor/EditorModule.cpp"
          "Src/SimRobotEditor/EditorModule.h"
          "Src/SimRobotEditor/EditorWidget.cpp"
          "Src/SimRobotEditor/EditorWidget.h"
          "Src/SimRobotEditor/SimRobotEditor.h"
          "Src/SimRobotEditor/SimRobotEditor.qrc"
          "Src/SimRobotEditor/SyntaxHighlighter.cpp"
          "Src/SimRobotEditor/SyntaxHighlighter.h")

set_target_properties(SimRobotEditor PROPERTIES AUTOMOC ON AUTORCC ON)

target_link_libraries(SimRobotEditor PUBLIC ${Qt_LIBRARIES})

target_include_directories(SimRobotEditor PRIVATE "Src/SimRobotEditor")
target_link_libraries(SimRobotEditor PUBLIC ${Qt_LIBRARIES})

# ##############################################################################
# SimRobotHelp ###
# ##############################################################################

target_sources(
  SimRobotHelp
  PRIVATE "Src/SimRobotHelp/HelpModule.cpp" "Src/SimRobotHelp/HelpModule.h"
          "Src/SimRobotHelp/HelpWidget.cpp" "Src/SimRobotHelp/HelpWidget.h"
          "Src/SimRobotHelp/SimRobotHelp.qrc")
target_include_directories(SimRobotHelp PRIVATE "Src/SimRobotHelp/")
set_target_properties(SimRobotHelp PROPERTIES AUTOMOC ON AUTORCC ON)
target_link_libraries(SimRobotHelp PUBLIC ${Qt_LIBRARIES})

# ##############################################################################
# SimRobot ###
# ##############################################################################

target_sources(
  SimRobot
  PRIVATE "Src/SimRobot/Main.cpp"
          "Src/SimRobot/RegisteredDockWidget.cpp"
          "Src/SimRobot/RegisteredDockWidget.h"
          "Src/SimRobot/resource.h"
          "Src/SimRobot/SceneGraphDockWidget.cpp"
          "Src/SimRobot/SceneGraphDockWidget.h"
          "Src/SimRobot/SimRobot.h"
          "Src/SimRobot/SimRobot.qrc"
          "Src/SimRobot/SimRobotWindow.cpp"
          "Src/SimRobot/SimRobotWindow.h"
          "Src/SimRobot/StatusBar.cpp"
          "Src/SimRobot/StatusBar.h")

set_target_properties(SimRobot PROPERTIES AUTOMOC ON AUTORCC ON)

target_link_libraries(SimRobot PRIVATE ${Qt_LIBRARIES})
target_link_libraries(SimRobot PUBLIC SimRobotCore2 SimRobotEditor SimRobotHelp)

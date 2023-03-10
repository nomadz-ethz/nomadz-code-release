target_sources(
  Controller
  PRIVATE "BHToolBar.cpp"
          "BHToolBar.h"
          "CameraCalibratorHandler.cpp"
          "CameraCalibratorHandler.h"
          "ConsoleRoboCupCtrl.cpp"
          "ConsoleRoboCupCtrl.h"
          "Controller.qrc"
          "ControllerRobot.cpp"
          "ControllerRobot.h"
          "GameController.cpp"
          "GameController.h"
          "ImageViewAdapter.cpp"
          "ImageViewAdapter.h"
          "LocalRobot.cpp"
          "LocalRobot.h"
          "LogPlayer.cpp"
          "LogPlayer.h"
          "Oracle.cpp"
          "Oracle.h"
          "Platform/Joystick.h"
          "Platform/Linux/Joystick.cpp"
          "Platform/Linux/Joystick.h"
          "RemoteRobot.cpp"
          "RemoteRobot.h"
          "Representations/Annotation.cpp"
          "Representations/Annotation.h"
          "Representations/Dataset.cpp"
          "Representations/Dataset.h"
          "Representations/Group.cpp"
          "Representations/Group.h"
          "Representations/Lesson.cpp"
          "Representations/Lesson.h"
          "Representations/ModuleInfo.cpp"
          "Representations/ModuleInfo.h"
          "Representations/TimeInfo.cpp"
          "Representations/TimeInfo.h"
          "RoboCupCtrl.cpp"
          "RoboCupCtrl.h"
          "RobotConsole.cpp"
          "RobotConsole.h"
          "TeamComm3DCtrl.cpp"
          "TeamComm3DCtrl.h"
          "TeamRobot.cpp"
          "TeamRobot.h"
          "Views/CABSLGraphView.cpp"
          "Views/CABSLGraphView.h"
          "Views/ColorCalibrationView/ColorCalibrationView.cpp"
          "Views/ColorCalibrationView/ColorCalibrationView.h"
          "Views/ColorCalibrationView/RangeSelector.cpp"
          "Views/ColorCalibrationView/RangeSelector.h"
          "Views/ColorCalibrationView/ThresholdSelector.cpp"
          "Views/ColorCalibrationView/ThresholdSelector.h"
          "Views/ColorReferenceView.cpp"
          "Views/ColorReferenceView.h"
          "Views/ColorSpaceView.cpp"
          "Views/ColorSpaceView.h"
          "Views/ConsoleView.cpp"
          "Views/ConsoleView.h"
          "Views/DataView/DataView.cpp"
          "Views/DataView/DataView.h"
          "Views/DataView/DataWidget.h"
          "Views/DataView/EditorEventFilter.cpp"
          "Views/DataView/EditorEventFilter.h"
          "Views/DataView/PropertyEditorFactory.cpp"
          "Views/DataView/PropertyEditorFactory.h"
          "Views/DataView/PropertyManager.cpp"
          "Views/DataView/PropertyManager.h"
          "Views/DataView/PropertyTreeCreator.cpp"
          "Views/DataView/PropertyTreeCreator.h"
          "Views/DataView/PropertyTreeWriter.cpp"
          "Views/DataView/PropertyTreeWriter.h"
          "Views/DataView/TypeDeclarations.h"
          "Views/DataView/TypeDescriptor.cpp"
          "Views/DataView/TypeDescriptor.h"
          "Views/DotView.cpp"
          "Views/DotView.h"
          "Views/FieldView.cpp"
          "Views/FieldView.h"
          "Views/FootView.cpp"
          "Views/FootView.h"
          "Views/ImageView.cpp"
          "Views/ImageView.h"
          "Views/JointView.cpp"
          "Views/JointView.h"
          "Views/ModuleGraphView.cpp"
          "Views/ModuleGraphView.h"
          "Views/PlotView.cpp"
          "Views/PlotView.h"
          "Views/SensorView.cpp"
          "Views/SensorView.h"
          "Views/StateMachineBehaviorView.cpp"
          "Views/StateMachineBehaviorView.h"
          "Views/TeachView/AnnotationItem.cpp"
          "Views/TeachView/AnnotationItem.h"
          "Views/TeachView/Annotator.cpp"
          "Views/TeachView/Annotator.h"
          "Views/TeachView/Classifier.cpp"
          "Views/TeachView/Classifier.h"
          "Views/TeachView/ClassifierView.cpp"
          "Views/TeachView/ClassifierView.h"
          "Views/TeachView/DatasetView.cpp"
          "Views/TeachView/DatasetView.h"
          "Views/TeachView/GroupCheckBox.cpp"
          "Views/TeachView/GroupCheckBox.h"
          "Views/TeachView/GroupView.cpp"
          "Views/TeachView/GroupView.h"
          "Views/TeachView/PixmapTile.cpp"
          "Views/TeachView/PixmapTile.h"
          "Views/TeachView/Sampler.cpp"
          "Views/TeachView/Sampler.h"
          "Views/TeachView/SampleView.cpp"
          "Views/TeachView/SampleView.h"
          "Views/TeachView/TeachSession.cpp"
          "Views/TeachView/TeachSession.h"
          "Views/TeachView/TeachView.cpp"
          "Views/TeachView/TeachView.h"
          "Views/TeachView/TeachWidget.cpp"
          "Views/TeachView/TeachWidget.h"
          "Views/TeachView/TileGridView.cpp"
          "Views/TeachView/TileGridView.h"
          "Views/TeachView/TrainView.cpp"
          "Views/TeachView/TrainView.h"
          "Views/TeamComm3DView.cpp"
          "Views/TeamComm3DView.h"
          "Views/TimeView/TimeView.cpp"
          "Views/TimeView/TimeView.h"
          "Views/TimeView/TimeWidget.cpp"
          "Views/TimeView/TimeWidget.h"
          "Views/VideoView.cpp"
          "Views/VideoView.h"
          "Views/View3D.cpp"
          "Views/View3D.h"
          "Views/ViewBike/BikeMenuBar.cpp"
          "Views/ViewBike/BikeMenuBar.h"
          "Views/ViewBike/TabWidget.cpp"
          "Views/ViewBike/TabWidget.h"
          "Views/ViewBike/ViewBike.cpp"
          "Views/ViewBike/ViewBikeGLWidget.cpp"
          "Views/ViewBike/ViewBikeGLWidget.h"
          "Views/ViewBike/ViewBike.h"
          "Views/ViewBike/ViewBikeMath.cpp"
          "Views/ViewBike/ViewBikeMath.h"
          "Views/ViewBike/ViewBikeWidget.cpp"
          "Views/ViewBike/ViewBikeWidget.h"
          "Visualization/DebugDrawing3D.cpp"
          "Visualization/DebugDrawing3D.h"
          "Visualization/DebugDrawing.cpp"
          "Visualization/DebugDrawing.h"
          "Visualization/HeaderedWidget.cpp"
          "Visualization/HeaderedWidget.h"
          "Visualization/OpenGLMethods.cpp"
          "Visualization/OpenGLMethods.h"
          "Visualization/PaintMethods.cpp"
          "Visualization/PaintMethods.h"
          "Visualization/VideoPlayer.cpp"
          "Visualization/VideoPlayer.h"
          "${SRC_DIR}/Apps/URC/MofCompiler.cpp")

target_precompile_headers(Controller REUSE_FROM representations)

set_target_properties(
  Controller
  PROPERTIES AUTOMOC ON
             AUTORCC ON
             POSITION_INDEPENDENT_CODE ON)

target_include_directories(
  Controller
  PRIVATE ${SRC_DIR}
          "${SRC_DIR}/Apps/SimRobot/Src/SimRobot/"
          "${SRC_DIR}/Apps/SimRobot/Src/SimRobotCore2/"
          "${SRC_DIR}/Apps/SimRobot/Src/SimRobotEditor/"
          "${SRC_DIR}/Apps/SimRobot/Src/SimRobotHelp/"
          "${SRC_DIR}/Apps/URC/include")

target_compile_definitions(Controller PRIVATE TARGET_SIM)

target_link_libraries(
  Controller PUBLIC snappy ${Qt_LIBRARIES} ${OpenCV_LIBS} qtpropertybrowser qxt
                    ${OPENGL_LIBRARIES})

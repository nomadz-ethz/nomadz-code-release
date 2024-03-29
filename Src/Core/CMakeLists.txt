# ##############################################################################
# Core library ###
# ##############################################################################

add_library(core OBJECT "")

target_sources(
  core
  PRIVATE "ColorClasses.h"
          "Communication/RoboCupControlData.h"
          "Debugging/Asserts.h"
          "Debugging/DebugDataStreamer.cpp"
          "Debugging/DebugDataStreamer.h"
          "Debugging/DebugDataTable.cpp"
          "Debugging/DebugDataTable.h"
          "Debugging/DebugDrawings3D.h"
          "Debugging/DebugDrawings.cpp"
          "Debugging/DebugDrawings.h"
          "Debugging/Debugging.h"
          "Debugging/DebugImages.h"
          "Debugging/DebugRequest.cpp"
          "Debugging/DebugRequest.h"
          "Debugging/DrawingHelpers.h"
          "Debugging/Modify.h"
          "Debugging/QueueFillRequest.cpp"
          "Debugging/QueueFillRequest.h"
          "Debugging/Stopwatch.h"
          "Debugging/TcpConnection.cpp"
          "Debugging/TcpConnection.h"
          "Debugging/TimingManager.cpp"
          "Debugging/TimingManager.h"
          "Debugging/Watch.h"
          "Enum.cpp"
          "Enum.h"
          "Function.h"
          "Global.cpp"
          "Global.h"
          "Math/Angle.h"
          "Math/Approx.h"
          "Math/BHMath.h"
          "Math/Common.h"
          "Math/Constants.h"
          "Math/ConvexHull.h"
          "Math/Covariance.h"
          "Math/EigenArrayExtensions.h"
          "Math/Eigen.h"
          "Math/EigenMatrixBaseExtensions.h"
          "Math/GaussianDistribution2D.cpp"
          "Math/GaussianDistribution2D.h"
          "Math/GaussianDistribution3D.cpp"
          "Math/GaussianDistribution3D.h"
          "Math/GaussianDistribution.cpp"
          "Math/GaussianDistribution.h"
          "Math/GaussianTable.h"
          "Math/Kalman.h"
          "Math/Matrix2x2.h"
          "Math/Matrix3x3.h"
          "Math/Matrix.cpp"
          "Math/Matrix.h"
          "Math/MVTools.cpp"
          "Math/MVTools.h"
          "Math/Pose2D.cpp"
          "Math/Pose2D.h"
          "Math/Pose3D.h"
          "Math/Probabilistics.h"
          "Math/Random.h"
          "Math/Rotation.h"
          "Math/RotationMatrix.cpp"
          "Math/RotationMatrix.h"
          "Math/SpatialInertiaDerivative.h"
          "Math/SpatialInertia.h"
          "Math/SpatialTransform.h"
          "Math/SpatialVector.h"
          "Math/Vector2.h"
          "Math/Vector3.h"
          "Math/Vector.h"
          "Math/YaMatrix.h"
          "MessageQueue/InMessage.cpp"
          "MessageQueue/InMessage.h"
          "MessageQueue/MessageIDs.h"
          "MessageQueue/MessageQueueBase.cpp"
          "MessageQueue/MessageQueueBase.h"
          "MessageQueue/MessageQueue.cpp"
          "MessageQueue/MessageQueue.h"
          "MessageQueue/OutMessage.cpp"
          "MessageQueue/OutMessage.h"
          "Module/Blackboard.cpp"
          "Module/Blackboard.h"
          "Module/Module.cpp"
          "Module/Module.h"
          "Module/ModuleManager.cpp"
          "Module/ModuleManager.h"
          "Module/ModulePackage.h"
          "ProcessFramework/PlatformProcess.h"
          "ProcessFramework/Process.cpp"
          "ProcessFramework/ProcessFramework.cpp"
          "ProcessFramework/ProcessFramework.h"
          "ProcessFramework/Process.h"
          "ProcessFramework/Receiver.cpp"
          "ProcessFramework/Receiver.h"
          "ProcessFramework/Sender.cpp"
          "ProcessFramework/Sender.h"
          "ProcessFramework/TeamHandler.cpp"
          "ProcessFramework/TeamHandler.h"
          "Range.h"
          "RangeNew.h"
          "RingBuffer.h"
          "RingBufferNew.h"
          "RingBufferWithSum.h"
          "RingBufferWithSumNew.h"
          "SensorData.h"
          "Settings.cpp"
          "Settings.h"
          "Streams/AutoStreamable.h"
          "Streams/Eigen.h"
          "Streams/EnumIndexedArray.h"
          "Streams/FieldWrapper.h"
          "Streams/InOut.cpp"
          "Streams/InOut.h"
          "Streams/InStreams.cpp"
          "Streams/InStreams.h"
          "Streams/OutStreams.cpp"
          "Streams/OutStreams.h"
          "Streams/SimpleMap.cpp"
          "Streams/SimpleMap.h"
          "Streams/Streamable.cpp"
          "Streams/Streamable.h"
          "Streams/StreamableHelper.h"
          "Streams/StreamHandler.cpp"
          "Streams/StreamHandler.h"
          "System/AlignedMemory.cpp"
          "System/AlignedMemory.h"
          "System/BHAssert.cpp"
          "System/BHAssert.h"
          "System/DebugHandler.cpp"
          "System/DebugHandler.h"
          "System/Directory.cpp"
          "System/Directory.h"
          "System/File.cpp"
          "System/File.h"
          "System/Function.h"
          "System/Semaphore.cpp"
          "System/Semaphore.h"
          "System/SharedMemory.cpp"
          "System/SharedMemory.h"
          "System/SoundPlayer.cpp"
          "System/SoundPlayer.h"
          "System/SystemCall.cpp"
          "System/SystemCall.h"
          "System/TcpComm.cpp"
          "System/TcpComm.h"
          "System/Thread.h"
          "System/Time.cpp"
          "System/Time.h"
          "System/UdpComm.cpp"
          "System/UdpComm.h")

if(NOMADZ_TARGET STREQUAL "host")
  target_compile_definitions(core PRIVATE TARGET_SIM)
else()
  target_sources(core PRIVATE "${SRC_DIR}/Nao/NaoBody.cpp"
                              "${SRC_DIR}/Nao/NaoBody.h")
  target_compile_definitions(core PRIVATE TARGET_ROBOT)

  install(
    DIRECTORY "${CMAKE_SOURCE_DIR}/Src/Nao"
    DESTINATION "include/nomadz_legacy"
    FILES_MATCHING
    PATTERN "NaoBody*")
endif()

if(NOMADZ_NG
   OR NOMADZ_TARGET STREQUAL "host")
  set_property(TARGET core PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()

target_include_directories(
  core PUBLIC ${SRC_DIR} ${SRC_DIR}/Apps/naobridge/include
              ${Eigen3_INCLUDE_DIRS} ${GAME_CONTROLLER_INCLUDE_DIRS})

target_link_libraries(
  core
  PUBLIC gc_tc_headers Threads::Threads
  PRIVATE rt)

install(
  DIRECTORY "${CMAKE_SOURCE_DIR}/Src/Core"
  DESTINATION "include/nomadz_legacy"
  FILES_MATCHING
  PATTERN "*.h")

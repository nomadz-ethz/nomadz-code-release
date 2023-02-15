target_sources(
  bush
  PRIVATE bush.cpp
          bush.qrc
          models/Team.cpp
          models/Power.cpp
          models/Robot.cpp
          cmds/SSHCmd.cpp
          cmds/ShowCmd.cpp
          cmds/SCPCmd.cpp
          cmds/RestartCmd.cpp
          cmds/DeployCmd.cpp
          cmds/DebugRequestCmd.cpp
          cmds/ExitCmd.cpp
          cmds/PingCmd.cpp
          cmds/CompileCmd.cpp
          cmds/SimCmd.cpp
          cmds/UpdateWirelessCmd.cpp
          cmds/ChangeWirelessCmd.cpp
          cmds/DownloadLogsCmd.cpp
          cmds/UpdateSettingsCmd.cpp
          cmds/HelpCmd.cpp
          cmds/ShutdownCmd.cpp
          cmdlib/CommandAdapter.cpp
          cmdlib/RobotCommand.cpp
          cmdlib/Context.cpp
          cmdlib/ProcessRunner.cpp
          cmdlib/AbstractConsole.cpp
          cmdlib/SaveConfigHelper.cpp
          cmdlib/Commands.cpp
          tools/ConfigMapParser.cpp
          tools/ShellTools.cpp
          tools/StringTools.cpp
          tools/ConfigMap.cpp
          tools/Platform.cpp
          tools/Filesystem.cpp
          ui/ShortcutBar.cpp
          ui/Console.cpp
          ui/TeamView.cpp
          ui/CommandLineEdit.cpp
          ui/CommandLineCompleter.cpp
          ui/RobotView.cpp
          ui/MainWindow.cpp
          ui/TeamSelector.cpp
          ui/VisualContext.cpp
          ui/RobotPool.cpp
          Session.cpp
          Initializer.cpp
          agents/PingAgent.cpp
          agents/TeamCommAgent.cpp
          agents/RemoteRobotAgent.cpp
          agents/StatusAgent.cpp
          agents/StatusAgent.h
          bhwrapper/TeamCommWrapper.cpp
          bhwrapper/Framework.cpp)
target_include_directories(bush PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if(NOMADZ_NG)
  target_compile_definitions(bush PRIVATE BUILD_COLCON)
  target_compile_definitions(bush PRIVATE COLCON_INSTALL_DIR=\"${CMAKE_INSTALL_PREFIX}/\")
  target_compile_definitions(bush PRIVATE COLCON_ROBOT_WORKSPACE_DIR=\"${CMAKE_INSTALL_PREFIX}/robot-workspace/\")
  target_compile_definitions(bush PRIVATE COLCON_NOMADZ_LEGACY_PACKAGE_DIR=\"${CMAKE_SOURCE_DIR}\")
else()
  target_compile_definitions(bush PRIVATE BUILD_CMAKE)
endif()
target_compile_definitions(bush PRIVATE TARGET_TOOL)
set_target_properties(bush PROPERTIES AUTOMOC ON AUTORCC ON)

target_link_libraries(bush PRIVATE core ${CMAKE_DL_LIBS} ${Qt_LIBRARIES})

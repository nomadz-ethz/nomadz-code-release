/**
 * @file SimRobot/Main.cpp
 * Implementation of the main function of SimRobot
 * @author Colin Graf
 */

#include <QtGlobal>
#include <QApplication>
#include <QTextCodec>

#ifdef ENABLE_ROS_TARGET
#include <rclcpp/rclcpp.hpp>
#endif

#ifdef _WIN32
#include "qtdotnetstyle.h"
#endif
#include "SimRobotWindow.h"

#ifdef MACOSX
#include <QFileOpenEvent>
#include "MacFullscreen.h"

class SimRobotApp : public QApplication {
public:
  SimRobotApp(int& argc, char** argv) : QApplication(argc, argv) {}

  MainWindow* mainWindow;

protected:
  bool event(QEvent* ev) {
    if (ev->type() == QEvent::FileOpen) {
      mainWindow->openFile(static_cast<QFileOpenEvent*>(ev)->file());
      return true;
    } else
      return QApplication::event(ev);
  }
};

#define QApplication SimRobotApp
#endif

int main(int argc, char* argv[]) {
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  QTextCodec::setCodecForTr(QTextCodec::codecForName("UTF-8"));
#endif
#ifdef _WIN32
  QApplication::setStyle(new QtDotNetStyle(QtDotNetStyle::Standard));
#endif
  QApplication app(argc, argv);
  MainWindow mainWindow(argc, argv);
#ifdef MACOSX
#ifdef MAC_OS_X_VERSION_10_9
  QFont::insertSubstitution(".Lucida Grande UI", "Lucida Grande");
#endif
  app.mainWindow = &mainWindow;
  app.setStyle("macintosh");
  MacFullscreen::enable(&mainWindow);
#endif
  app.setApplicationName("SimRobot");

#ifdef ENABLE_ROS_TARGET
  rclcpp::init(argc, argv);
#endif
  mainWindow.show();

  // open file from commandline
  for (int i = 1; i < argc; i++)
    if (*argv[i] != '-') {
#ifdef MACOSX
      if (strcmp(argv[i], "YES")) {
        mainWindow.setWindowOpacity(0);
        mainWindow.show();
        mainWindow.openFile(argv[i]);
        mainWindow.setWindowOpacity(1);
      }
#else
      mainWindow.openFile(argv[i]);
#endif
      break;
    }

#ifdef ENABLE_ROS_TARGET
  rclcpp::uninstall_signal_handlers();
#endif

  int result = app.exec();

#ifdef ENABLE_ROS_TARGET
  rclcpp::shutdown();
#endif

#ifdef LINUX
  exit(result); // fixes a mysterious segfault
#endif
  return result;
}


#include "SaveConfigHelper.h"
#include "Session.h"
#include "ui/MainWindow.h"

void SaveConfigHelper::saveConfig() {
  /* save configuration. but we need to execute this on the UI thread :/
   * (because of access to Global:: )
   */
  if (firstRun) {
    connect(this,
            SIGNAL(saveConfiguration()),
            Session::getInstance().getMainWindow(),
            SLOT(saveConfiguration()),
            Qt::BlockingQueuedConnection);
    firstRun = false;
  }
  // emit saveConfiguration();   removed for OSX fix due to linker error (simomaur, 5.11.2016)
}

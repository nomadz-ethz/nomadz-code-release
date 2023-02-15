
#ifndef _SAVECONFIGHELPER_H_
#define _SAVECONFIGHELPER_H_

#include <QObject>

/** base class for a command that needs to save configuration changes in the UI
 * to the file system. This is a bit a delicate operation because commands
 * run in their own thread and saving configuration must run on the UI thread.
 */
class SaveConfigHelper : public QObject {
  Q_OBJECT
public:
protected:
  SaveConfigHelper() : firstRun(true) {}

  void saveConfig();

private:
  bool firstRun;
signals:
  void saveConfiguration();
};

#endif /* _SAVECONFIGHELPER_H_ */

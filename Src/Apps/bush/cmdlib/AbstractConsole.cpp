#include <QString>
#include "cmdlib/AbstractConsole.h"
#include "tools/StringTools.h"

AbstractConsole::AbstractConsole(QObject* parent) : QObject(parent), IConsole() {
  qRegisterMetaType<ConsolePrintTarget>();
}

void AbstractConsole::print(const std::string& msg) {
  emit sPrint(CPT_PRINT, fromString(msg));
}

void AbstractConsole::printLine(const std::string& msg) {
  emit sPrint(CPT_PRINT_LINE, fromString(msg));
}

void AbstractConsole::error(const std::string& msg) {
  emit sPrint(CPT_ERROR, fromString(msg));
}

void AbstractConsole::errorLine(const std::string& msg) {
  emit sPrint(CPT_ERROR_LINE, fromString(msg));
}

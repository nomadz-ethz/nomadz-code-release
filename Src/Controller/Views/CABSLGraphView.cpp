/**
 * @file CABSLGraphView.cpp
 *
 * Simple script for creating a drawing of the option call graph of a CABSL behavior
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are Tim Laue and Colin Graf
 */

#include <QFile>
#include <QTextStream>

#include "Core/System/File.h"
#include "CABSLGraphView.h"

CABSLGraphViewObject::Option::Option(const QString& line) {
  originalLine = line;
  filePath = QString(line).replace("#include ", "");
  filePath = filePath.trimmed();
  filePath.remove(QChar('\"'));
  QStringList path = filePath.split(QChar('/'));
  QString file = path.last();
  optionName = file.split(QChar('.')).first();
  dotColorDesc = "";
  if (path.contains("Skills")) {
    dotColorDesc = "[style=filled, fillcolor=khaki1, color=yellow4]";
  } else if (path.contains("GameControl")) {
    dotColorDesc = "[style=filled, fillcolor=pink, color=red]";
  } else if (path.contains("HeadControl")) {
    dotColorDesc = "[style=filled, fillcolor=tan, color=saddlebrown]";
  } else if (path.contains("Output")) {
    dotColorDesc = "[style=filled, fillcolor=lightskyblue1, color=navyblue]";
  } else if (path.contains("Tools")) {
    dotColorDesc = "[style=filled, fillcolor=lightgray, color=black]";
  } else if (path.contains("Roles")) {
    dotColorDesc = "[style=filled, fillcolor=palegreen, color=mediumseagreen]";
  } else {
    dotColorDesc = "";
  }
}

void CABSLGraphViewObject::Option::findCalls(const QString& behaviorName, QList<Option*>& options) {
  QString fileName = QString(File::getBHDir()) + "/Src/Modules/BehaviorControl/" + behaviorName + "/" + filePath;
  QFile file(fileName);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return;
  }
  QTextStream in(&file);
  while (!in.atEnd()) {
    QString line = in.readLine();
    for (Option* option : options) {
      if (line.contains(option->pattern())) {
        if (!calls.contains(option->optionName)) {
          calls << option->optionName;
        }
      }
    }
  }
}

QString CABSLGraphViewObject::Option::declareNode() {
  return optionName + dotColorDesc + ";\n";
}

QString CABSLGraphViewObject::Option::drawCalls() {
  QString dotLine;
  for (QString call : calls) {
    dotLine += optionName + " -> " + call + ";\n";
  }
  return dotLine;
}

QString CABSLGraphViewObject::Option::pattern() {
  return " " + optionName + "(";
}

CABSLGraphViewObject::CABSLGraphViewObject(const QString& fullName, const QString& behaviorName, const QString& optionsFile)
    : DotViewObject(fullName), behaviorName(behaviorName), optionsFile(optionsFile) {}

QString CABSLGraphViewObject::generateDotFileContent() {
  QList<Option*> options;

  // Read files and find option calls
  QString fileName = QString(File::getBHDir()) + "/Src/Modules/BehaviorControl/" + behaviorName + "/" + optionsFile;
  QFile file(fileName);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return QString();
  }
  // Read all lines from global options file
  QTextStream in(&file);
  while (!in.atEnd()) {
    QString line = in.readLine();
    if (line.contains(".h") && line.contains("#include")) {
      options << new Option(line);
    }
  }
  // Find calls of other options
  for (Option* option : options) {
    option->findCalls(behaviorName, options);
  }

  // Create dot file
  QString dotSource = "digraph G {\n";
  for (Option* option : options) {
    dotSource += option->declareNode();
  }
  for (Option* option : options) {
    dotSource += option->drawCalls();
  }
  dotSource += "}\n";

  qDeleteAll(options);
  return dotSource;
}

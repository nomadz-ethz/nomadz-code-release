#include <QFormLayout>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPalette>
#include <QPushButton>
#include <QResizeEvent>
#include <QScrollBar>
#include <QtCore>
#include <QApplication>
#include <QtGlobal>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtConcurrent/QtConcurrentRun>
#endif
#include "cmdlib/Context.h"
#include "tools/StringTools.h"
#include "ui/CommandLineEdit.h"
#include "ui/Console.h"
#include "ui/TeamSelector.h"
#include "ui/VisualContext.h"

Icons Icons::theIcons;

void Icons::init() {
  ICON_GRAY = QIcon(":icons/gray.png");
  ICON_GREEN = QIcon(":icons/green.png");
  ICON_ORANGE = QIcon(":icons/orange.png");
  ICON_RED = QIcon(":icons/red.png");
}

VisualContextDecoration::VisualContextDecoration(const QString& commandLine, VisualContext* parent, VisualContext* context)
    : QFrame(parent), button(new QPushButton(Icons::getInstance().ICON_GRAY, "", this)), header(new QLabel(commandLine)),
      visualContext(context), parentContext(parent) {
  setAutoFillBackground(true);
  QPalette p = palette();
  p.setColor(QPalette::Background, p.color(QPalette::AlternateBase));
  setPalette(p);
  QFormLayout* layout = new QFormLayout();
  layout->setSpacing(3);
  header->setFrameStyle(QFrame::Box);
  layout->addRow(button, header);
  button->setMaximumWidth(25);
  button->setFlat(true);
  button->setCheckable(true);
  button->setChecked(true);
  layout->addRow(visualContext);
  setLayout(layout);

  connect(visualContext, SIGNAL(statusChanged(bool)), this, SLOT(updateStatus(bool)));
  connect(visualContext, SIGNAL(sCanceled()), this, SLOT(canceled()));
}

void VisualContextDecoration::updateStatus(bool status) {
  if (button->icon().cacheKey() != Icons::getInstance().ICON_ORANGE.cacheKey()) {
    if (status)
      button->setIcon(Icons::getInstance().ICON_GREEN);
    else
      button->setIcon(Icons::getInstance().ICON_RED);
  }
}

void VisualContextDecoration::canceled() {
  button->setIcon(Icons::getInstance().ICON_ORANGE);
}

ScrollArea::ScrollArea(QWidget* parent) : QScrollArea(parent), scrollEnabled(true) {
  connect(verticalScrollBar(), SIGNAL(valueChanged(int)), this, SLOT(updateScrollEnabled()));
}

bool ScrollArea::viewportEvent(QEvent* event) {
  bool ret = QScrollArea::viewportEvent(event);
  if (event->type() == QEvent::LayoutRequest && widget() && scrollEnabled)
    ensureVisible(0, widget()->size().height());
  return ret;
}

void ScrollArea::updateScrollEnabled() {
  scrollEnabled = verticalScrollBar()->value() == verticalScrollBar()->maximum();
}

Console::Console(TeamSelector* teamSelector)
    : visualContext(new VisualContext(this)), teamSelector(teamSelector), scrollArea(new ScrollArea(this)), prompt(0),
      cmdLine(0) {
  cmdLine = new CommandLineEdit(this);

  prompt = new QLabel("bush>", cmdLine);
  prompt->setAutoFillBackground(true);
  QPalette p = prompt->palette();
  p.setColor(QPalette::Background, p.color(QPalette::AlternateBase));
  prompt->setPalette(p);

  QGridLayout* layout = new QGridLayout();
  layout->setHorizontalSpacing(0);
  scrollArea->setWidget(visualContext);
  scrollArea->setWidgetResizable(true);
  scrollArea->setBackgroundRole(QPalette::AlternateBase);
  layout->addWidget(scrollArea, 0, 0);
  layout->setRowStretch(0, 1);
  QFormLayout* fl = new QFormLayout();
  fl->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
  fl->addRow(prompt, cmdLine);
  layout->addLayout(fl, 1, 0);
  setLayout(layout);

  connect(cmdLine, SIGNAL(returnPressed()), this, SLOT(returnPressed()));
}

void Console::returnPressed() {
  fireCommand(cmdLine->text());
  cmdLine->setText("");
}

void Console::showEvent(QShowEvent* event) {
  cmdLine->setFocus();
  QFrame::showEvent(event);
}

void Console::fireCommand(const QString& command) {
  if (command.size() > 0) {
    QtConcurrent::run(visualContext, &VisualContext::executeInContext, this, teamSelector, command);
    cmdLine->setFocus();
  }
}

void Console::cancel() {
  visualContext->cancel();
}

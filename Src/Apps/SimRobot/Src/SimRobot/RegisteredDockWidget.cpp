#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
#include <QCloseEvent>
#include <QMenu>
#include <QApplication>
#include <QClipboard>
#include <QSvgGenerator>
#include <QFileDialog>
#include <QPainter>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include "RegisteredDockWidget.h"
#include "SimRobotWindow.h"

RegisteredDockWidget::RegisteredDockWidget(const QString& fullName, QWidget* parent)
    : QDockWidget(parent), fullName(fullName), module(0), object(0), widget(0), flags(0), reallyVisible(false) {
  setObjectName(fullName);
  setAllowedAreas(Qt::TopDockWidgetArea);
  setFocusPolicy(Qt::ClickFocus);
  connect(this, SIGNAL(visibilityChanged(bool)), this, SLOT(visibilityChanged(bool)));
}

void RegisteredDockWidget::setWidget(SimRobot::Widget* widget,
                                     const SimRobot::Module* module,
                                     SimRobot::Object* object,
                                     int flags) {
  if (widget) {
#ifdef FIX_MACOSX_UNDOCKED_WIDGETS_DISAPPEAR_WHEN_DOCKED_BUG
    if (isFloating() && widget->getWidget()->inherits("QGLWidget")) {
      setFloating(false);
      QDockWidget::setWidget(widget->getWidget());
      setFloating(true);
    } else
#endif
      QDockWidget::setWidget(widget->getWidget());
  } else
    QDockWidget::setWidget(0);
  if (this->widget)
    delete this->widget;
  this->module = module;
  this->object = object;
  this->widget = widget;
  this->flags = flags;
}

bool RegisteredDockWidget::canClose() {
  return widget ? widget->canClose() : true;
}

QMenu* RegisteredDockWidget::createFileMenu() const {
  return widget ? widget->createFileMenu() : 0;
}

QMenu* RegisteredDockWidget::createEditMenu() {
  if (!widget)
    return 0;

  QMenu* menu = widget->createEditMenu();
  QAction* firstAction = 0;
  if (menu) {
    const QList<QAction*> actions = menu->actions();
    if (!actions.isEmpty())
      firstAction = actions.at(0);
  } else {
    menu = new QMenu(tr("&Edit"));
    flags |= SimRobot::Flag::copy;
  }

  if (flags & SimRobot::Flag::copy) {
    QAction* action = new QAction(QIcon(":/Icons/page_copy.png"), tr("&Copy"), 0);
    action->setShortcut(QKeySequence(QKeySequence::Copy));
    action->setStatusTip(tr("Copy the window drawing to the clipboard"));
    connect(action, SIGNAL(triggered()), this, SLOT(copy()));
    if (firstAction)
      menu->insertAction(firstAction, action);
    else
      menu->addAction(action);
  }

  return menu;
}

QMenu* RegisteredDockWidget::createUserMenu() const {
  if (!widget)
    return 0;

  QMenu* menu = widget->createUserMenu();

  if (menu) {

  } else if (flags & SimRobot::Flag::exportAsImage)
    menu = new QMenu(tr("&Object"));

  if (flags & SimRobot::Flag::exportAsImage) {
    QMenu* exportImgMenu = menu->addMenu(tr("&Export Image"));
    QAction* svgAction = exportImgMenu->addAction(tr("Export Image as &SVG"));
    QAction* pngAction = exportImgMenu->addAction(tr("Export Image as &PNG"));
    connect(svgAction, SIGNAL(triggered()), this, SLOT(exportAsSvg()));
    connect(pngAction, SIGNAL(triggered()), this, SLOT(exportAsPng()));
  }

  return menu;
}

void RegisteredDockWidget::update() {
  if (widget && reallyVisible)
    widget->update();
}

QAction* RegisteredDockWidget::toggleViewAction() const {
  QAction* action = QDockWidget::toggleViewAction();
  if (object) {
    const QIcon* icon = object->getIcon();
    if (icon)
      action->setIcon(*icon);
  }
  return action;
}

void RegisteredDockWidget::closeEvent(QCloseEvent* event) {
  if (!canClose()) {
    event->ignore();
    return;
  }

#ifdef FIX_MACOSX_UNDOCKED_WIDGETS_DURING_CLOSE_BUG
  setFloating(false);
#endif
  QDockWidget::closeEvent(event);

  emit closedObject(fullName);
}

void RegisteredDockWidget::contextMenuEvent(QContextMenuEvent* event) {
  if (!widget) {
    QDockWidget::contextMenuEvent(event);
    return;
  }

  QRect content(QDockWidget::widget()->geometry());
  if (!content.contains(event->x(), event->y())) { // click on window frame
    QDockWidget::contextMenuEvent(event);
    return;
  };

  // try to show context menu
  QMenu menu;
  QMenu* editMenu = createEditMenu();
  QMenu* userMenu = createUserMenu();
  QMenu* simMenu = ((MainWindow*)MainWindow::application)->createSimMenu();
  if (editMenu) {
    QMetaObject::invokeMethod(editMenu, "aboutToShow", Qt::DirectConnection);
    const QList<QAction*> actions = editMenu->actions();
    for (QAction* action : actions) {
      editMenu->removeAction(action);
      menu.addAction(action);
    }
    menu.addSeparator();
  }
  menu.addAction(simMenu->menuAction());
  if (userMenu) {
    QMetaObject::invokeMethod(userMenu, "aboutToShow", Qt::DirectConnection);
    menu.addSeparator();
    const QList<QAction*> actions = userMenu->actions();
    for (QAction* action : actions) {
      userMenu->removeAction(action);
      menu.addAction(action);
    }
  }
  event->accept();
  QAction* action = menu.exec(mapToGlobal(QPoint(event->x(), event->y())));
  delete simMenu;
  if (editMenu)
    delete editMenu;
  if (userMenu)
    delete userMenu;

  if (action)
    emit closedContextMenu();
}

void RegisteredDockWidget::visibilityChanged(bool visible) {
  reallyVisible = visible;
}

void RegisteredDockWidget::copy() {
  QApplication::clipboard()->clear();
  QApplication::clipboard()->setPixmap(QPixmap::grabWidget(QDockWidget::widget()));
}

void RegisteredDockWidget::exportAsSvg() {
  if (!widget)
    return;

  QSettings& settings = MainWindow::application->getSettings();
  QString fileName = QFileDialog::getSaveFileName(
    this, tr("Export as SVG"), settings.value("ExportDirectory", "").toString(), tr("Scalable Vector Graphics (*.svg)"));
  if (fileName.isEmpty())
    return;
  settings.setValue("ExportDirectory", QFileInfo(fileName).dir().path());

  QSize size = widget->getWidget()->sizeHint();
  QSvgGenerator generator;
  generator.setFileName(fileName);
  generator.setSize(size);
  generator.setViewBox(QRect(0, 0, size.width(), size.height()));
  generator.setTitle(windowTitle());
  generator.setDescription(tr("An SVG drawing created by SimRobot."));
  QPainter painter;
  painter.begin(&generator);
  painter.setClipRect(QRect(0, 0, size.width(), size.height()));
  widget->paint(painter);
  painter.end();
}

void RegisteredDockWidget::exportAsPng() {
  if (!widget)
    return;

  QSettings& settings = MainWindow::application->getSettings();
  QString fileName =
    QFileDialog::getSaveFileName(this, tr("Export as PNG"), settings.value("ExportDirectory", "").toString(), tr("(*.png)"));
  if (fileName.isEmpty())
    return;
  settings.setValue("ExportDirectory", QFileInfo(fileName).dir().path());

  QPixmap pixmap(widget->getWidget()->size());
  widget->getWidget()->render(&pixmap);
  pixmap.save(fileName, "PNG");
}

void RegisteredDockWidget::keyPressEvent(QKeyEvent* event) {
  if (isFloating() &&
      (event->modifiers() & (Qt::ControlModifier | Qt::ShiftModifier)) == (Qt::ControlModifier | Qt::ShiftModifier)) {
    ((RegisteredDockWidget*)qobject_cast<QWidget*>(parent()))->keyPressEvent(event);
    // note: the dirty cast is for accessing keyPressEvent
    if (event->isAccepted())
      return;
  }

  QDockWidget::keyPressEvent(event);
}

void RegisteredDockWidget::keyReleaseEvent(QKeyEvent* event) {
  if (isFloating() &&
      (event->modifiers() & (Qt::ControlModifier | Qt::ShiftModifier)) == (Qt::ControlModifier | Qt::ShiftModifier)) {
    ((RegisteredDockWidget*)qobject_cast<QWidget*>(parent()))->keyReleaseEvent(event);
    // note: the dirty cast is for accessing keyReleaseEvent
    if (event->isAccepted())
      return;
  }

  QDockWidget::keyReleaseEvent(event);
}

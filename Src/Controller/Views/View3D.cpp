/**
 * @file View3D.cpp
 *
 * Implementation of class View3D
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a> and Colin Graf
 */

#include "Platform/OpenGL.h"
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
#include <QGLWidget>
#include <QMouseEvent>
#include <QSettings>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include "View3D.h"
#include "Controller/RoboCupCtrl.h"

class View3DWidget : public QGLWidget, public SimRobot::Widget {
public:
  View3DWidget(View3D& view3D) : view3D(view3D), dragging(false) {
    setFocusPolicy(Qt::StrongFocus);

    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(view3D.fullName);
    rotation = settings.value("Rotation").toPointF();
    settings.endGroup();
  }

  virtual ~View3DWidget() {
    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(view3D.fullName);
    settings.setValue("Rotation", rotation);
    settings.endGroup();
  }

private:
  void resizeGL(int newWidth, int newHeight) {
    width = newWidth;
    height = newHeight;
  }

  void paintGL() {
    GLdouble aspect = height ? (GLdouble)width / (GLdouble)height : (GLdouble)width;

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glLineWidth(1.5); // required
    glPointSize(2.5);
    glPolygonMode(GL_FRONT, GL_LINE);
    glPolygonMode(GL_BACK, GL_LINE);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_DEPTH_TEST);

    if (!glIsList(View3D::cubeId) || view3D.needsUpdate()) {
      view3D.updateDisplayLists();
    }

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glClearColor(view3D.background.x, view3D.background.y, view3D.background.z, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(25, aspect, 1, 100);

    glTranslated(0.0f, 0.0f, -view3D.getViewDistance());
    glRotated(rotation.x(), 1.0f, 0.0f, 0.0f);
    glRotated(rotation.y(), 0.0f, 0.0f, 1.0f);

    glCallList(View3D::cubeId);
    glCallList(View3D::colorsId);

    view3D.lastBackground = view3D.background;
  }

  void mousePressEvent(QMouseEvent* event) {
    QWidget::mousePressEvent(event);

    if (event->button() == Qt::LeftButton || event->button() == Qt::MidButton) {
      dragStart = event->pos();
      dragging = true;
    }
  }

  void mouseReleaseEvent(QMouseEvent* event) {
    QWidget::mouseReleaseEvent(event);

    dragging = false;
  }

  void mouseMoveEvent(QMouseEvent* event) {
    QWidget::mouseMoveEvent(event);

    if (dragging) {
      QPoint diff(event->pos() - dragStart);
      dragStart = event->pos();
      rotation.ry() += diff.x();
      rotation.rx() += diff.y();
      updateGL();
    }
  }

  void mouseDoubleClickEvent(QMouseEvent* event) {
    QWidget::mouseDoubleClickEvent(event);

    rotation = QPointF();
    updateGL();
  }

  virtual QSize sizeHint() const { return QSize(320, 240); }

  virtual QWidget* getWidget() { return this; }

  virtual void update() {
    if (view3D.background != view3D.lastBackground || view3D.needsUpdate()) {
      QGLWidget::update();
    }
  }

  QPointF rotation;
  int width;
  int height;
  View3D& view3D;
  bool dragging;
  QPoint dragStart;
};

View3D::View3D(const QString& fullName, const Vector3<>& background)
    : background(background), fullName(fullName), icon(":/Icons/tag_green.png") {}

SimRobot::Widget* View3D::createWidget() {
  return new View3DWidget(*this);
}

/**
 * @file EditorEventFilter.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Arne Böckmann (arneboe@tzi.de)
 */

#pragma once

#include <QObject>

class QEvent;
class DataView;
class QWidget;
class QtProperty;

class EditorEventFilter : public QObject {
public:
  /**
   * @param The editor will forward all events to the DataView.
   * @param pSource the editor which is the source of all events.
   * @param pProperty The property which belongs to the editor.
   */
  EditorEventFilter(QObject* pParent, DataView* pView, QWidget* pSource, QtProperty* pProperty);

protected:
  bool eventFilter(QObject* obj, QEvent* event);

private:
  DataView* pTheView;
  QWidget* pTheSource;
  QtProperty* pTheProperty;
};

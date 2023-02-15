/**
 * @file EditorEventFilter.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Arne Böckmann (arneboe@tzi.de)
 */

#include "EditorEventFilter.h"
#include <QEvent>
#include "DataView.h"

EditorEventFilter::EditorEventFilter(QObject* pParent, DataView* pView, QWidget* pSource, QtProperty* pProperty)
    : QObject(pParent), pTheView(pView), pTheSource(pSource), pTheProperty(pProperty) {}

bool EditorEventFilter::eventFilter(QObject* obj, QEvent* event) {
  if (NULL != pTheView &&
      (event->type() == QEvent::FocusIn || event->type() == QEvent::Paint || event->type() == QEvent::FocusOut ||
       event->type() == QEvent::LayoutRequest)) // prevent sending events after death
  {
    return pTheView->handlePropertyEditorEvent(pTheSource, pTheProperty, event);
  } else {
    return false;
  }
}

/**
 * @file PropertyManager.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Arne Böckmann (arneboe@tzi.de)
 */

#pragma once

#include <qtvariantproperty.h>
#include <QMetaType>
#include "TypeDeclarations.h"
#include <QList>
#include <QMap>
#include <QVariant>
#include <QObject>
#include "TypeDescriptor.h"

using namespace Type;
/**
 * Extends the VariantPropertyManager to add support for additional datatypes.
 *
 * The QtPropertyBrowser does support most basic types out of the box.
 * However it only supports one kind of int/float. That is perfectly ok for
 * gui applications but we need to differentiate between the different kinds of
 * signed/unsigned/short/long ints etc.
 *
 * To understand what is happening here you should read:
 * http://doc.qt.nokia.com/qq/qq18-propertybrowser.html#extendingwithvariants
 *
 *
 * How to add new types:
 * (1) Create a TypeDescriptor for your type in TypeDescriptor.h
 * (2) Add it to initTypes().
 * (3) Register the datatype with qt in TypeDeclarations.h
 * (4) Modify PropertyEditorFactor::CreateEditor to provide an editor for your
 *     new type.
 * (5) Maybe you have to modify or add slots to the PropertyEditorFactory as well
 *
 */
class PropertyManager : public QtVariantPropertyManager {
  Q_OBJECT

public:
  PropertyManager();
  virtual ~PropertyManager();

  /**
   * @return The value of the specified property or an invalid QVariant of the
   *         property is not managed by this manager.
   */
  virtual QVariant value(const QtProperty* pProperty) const;

  /**
   * @return the type-id of the specified property.
   */
  virtual int valueType(int propertyType) const;

  /**
   * @return the value of the specified property as string or a default String of the property
   *         is not managed by this manager.
   */
  QString valueText(const QtProperty* property) const;

  /**
   * @return true if this VariantManager supports the type. False otherwise.
   */
  bool isPropertyTypeSupported(int type) const;

public slots:
  /**
   * Changes the value of the specified property.
   * Emits valueChanged and propertyChanged signals
   */
  virtual void setValue(QtProperty* property, const QVariant& val);

protected:
  /**
   * This method is called by the base class whenever a new property should
   * be created.
   */
  virtual void initializeProperty(QtProperty* property);

  /**
   * This method is called by the base class whenever a property should be
   * destroyed.
   */
  virtual void uninitializeProperty(QtProperty* property);

private slots:
  void slotValueChanged(QtProperty* property, const QVariant& value);

private:
  /**
   * Initialize information about the allowed types
   */
  void initTypes();

  /**
   * QtVariantProperties don't know anything about the type of the data they contain.
   * Therefore they cannot store the value internally. Instead the Manager stores
   * the values for all properties.
   */
  QMap<const QtProperty*, QVariant> theValues;

  /**
   * Mapping that assigns type descriptors to type ids.
   * key: type id
   * value: descriptor of that type
   */
  QMap<int, TypeDescriptor*> theDescriptors;
};

/**
 * @file PropertyTreeCreator.cpp
 *
 * The file declares a helper class to create and update the property tree.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Thomas Röfer
 */

#pragma once

#include <vector>
#include <cstdio>
#include "DataView.h"
#include "Core/System/BHAssert.h"
#include "Core/Streams/InOut.h"

/**
 * Helper class to create and update the property tree.
 */
class PropertyTreeCreator : public Out {
  /**
   * An entry representing the current state in the creation process.
   */
  class Entry {
  public:
    int type; /**< The type of the entry. -2: value or record, -1: array, >= 0: array element index. */
    const char* (*enumToString)(unsigned char); /**< A function that translates an enum to a string. */
    QtVariantProperty* parent;                  /**< The parent in the property tree. Can be 0 if there is none (yet). */
    QtVariantProperty* property; /**< The current node in the property tree if already created. Otherwise 0. */
    std::string path;            /**< The path to this node including its name. */
    std::string name;            /**< The name of this node. */

    /**
     * Constructor.
     * @param type The type of the entry. -2: value or record, -1: array, >= 0: array element index.
     * @param enumToString A function that translates an enum to a string.
     * @param parent The parent in the property tree. Can be 0 if there is none (yet).
     * @param path The path to this node including its name.
     * @param name The name of this node.
     */
    Entry(int type,
          const char* (*enumToString)(unsigned char),
          QtVariantProperty* parent,
          const std::string& path,
          const char* name)
        : type(type), enumToString(enumToString), parent(parent), property(0) {
      if (type >= 0) {
        char buf[20];
        sprintf(buf, "%d", type);
        this->name = buf;
      } else
        this->name = name;
      this->path = (path == "" ? "" : path + ".") + this->name;
    }
  };

  DataView& view;           /**< The data view using this object. */
  std::vector<Entry> stack; /**< The hierarchy of nodes that are created. */

protected:
  /** Helper to create a node. */
  template <class T> void out(const T& value) {
    Entry& e = stack.back();
    ASSERT(!e.property);
    e.property = view.getProperty(e.path, TypeDescriptor::getTypeId<T>(), e.name.c_str(), e.parent);
    e.property->setValue(value);
  }

  virtual void outBool(bool value) { out(value); }
  virtual void outChar(char value) { out((int)value); }
  virtual void outUChar(unsigned char value);
  virtual void outShort(short value) { out(value); }
  virtual void outUShort(unsigned short value) { out(value); }
  virtual void outInt(int value) { out(value); };
  virtual void outUInt(unsigned int value);
  virtual void outFloat(float value) { out(value); }
  virtual void outDouble(double value) { out(value); }
  virtual void outString(const char* value) { out(QString(value)); }
  virtual void outAngle(const Angle& value);
  virtual void outEndL() {}

public:
  QtVariantProperty* root; /**< The root of the property tree. */

  /**
   * Constructor.
   * @param view The data view using this object.
   */
  PropertyTreeCreator(DataView& view) : view(view), root(0) {}

  /**
   * Select an entry for writing.
   * @param name The name of the entry if type == -2, otherwise 0.
   * @param type The type of the entry.
   *             -2: value or record,
   *             -1: array,
   *             >= 0: array element index.
   * @param enumToString A function that translates an enum to a string.
   */
  virtual void select(const char* name, int type, const char* (*enumToString)(unsigned char));

  /** Deselects a field for writing. */
  virtual void deselect();

  /** Writing raw data is not supported. Do not call. */
  virtual void write(const void* p, int size) { ASSERT(false); }
};

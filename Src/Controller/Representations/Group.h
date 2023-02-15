/**
 * @file Group.h
 *
 * Declaration of class Group
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <QColor>
#include <QMetaType>
#include <QObject>
#include "Annotation.h"
#include "Dataset.h"
#include "Core/Streams/AutoStreamable.h"

class StreamableGroup;

class Group : public QObject {
public:
  Group(const Group& other) : name(other.name), color(other.color), members(other.members) {}

  Group(std::string name = "") : name(name), color(rand() % 256, rand() % 256, rand() % 256) {}

  Group(std::string name, const QColor& color) : name(name), color(color) {}

  Group(std::string name, const std::vector<Annotation*>& annotations) : Group(name) {
    for (Annotation* annotation : annotations) {
      members.insert(annotation);
    }
  }

  Group(std::string name, const std::vector<std::unique_ptr<Annotation>>& annotations) : Group(name) {
    for (const auto& annotation : annotations) {
      members.insert(annotation.get());
    }
  }

  Group(const StreamableGroup&, const std::unordered_map<int, Annotation*>&);

  static std::unordered_set<Annotation*> getAll(const std::vector<Group*>&);

  static std::unordered_set<Annotation*> getCircles(const std::vector<Group*>&);

  static std::unordered_set<Annotation*> getLines(const std::vector<Group*>&);

  static std::unordered_set<Annotation*> getPolygons(const std::vector<Group*>&);

  std::string name;
  QColor color;
  std::unordered_set<Annotation*> members;

  Group& operator+=(const Group& that) {
    members.insert(that.members.begin(), that.members.end());
    return *this;
  }

  Group operator+(const Group& that) const { return Group(*this) += that; }

  template <template <typename...> class C> void addMembers(const C<Annotation*>& added) {
    const size_t oldSize = members.size();

    std::copy(added.begin(), added.end(), std::inserter(members, members.begin()));

    if (oldSize != members.size()) {
      emit changed(this);
    }
  }

  void clearMembers() {
    if (!members.empty()) {
      members.clear();
      emit changed(this);
    }
  }

  template <template <typename...> class C> void removeMembers(const C<Annotation*>& removed) {
    const size_t oldSize = members.size();

    for (const auto& annotation : removed) {
      members.erase(annotation);
    }

    if (oldSize != members.size()) {
      emit changed(this);
    }
  }

  void setColor(const QColor& newColor) {
    if (color != newColor) {
      color = newColor;
      emit changed(this);
    }
  }

  void setName(const std::string& newName) {
    if (name != newName) {
      name = newName;
      emit changed(this);
    }
  }

  std::unordered_multimap<DataFrame*, Annotation*> getAnnotationsByFrame() const;

signals:
  void changed(Group*);

private:
  Q_OBJECT;
};

Q_DECLARE_METATYPE(Group*);

STREAMABLE(StreamableGroup, {
  public : StreamableGroup(int id, const Group&, const std::unordered_map<Annotation*, int>&),

  (int)id,
  (std::string)name,
  (std::string)color,
  (std::vector<int>)members,
});

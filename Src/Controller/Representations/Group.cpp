/**
 * @file Group.cpp
 *
 * Implementation of class Group
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include "Group.h"

Group::Group(const StreamableGroup& group, const std::unordered_map<int, Annotation*>& idToAnnotation)
    : name(group.name), color(group.color.c_str()) {
  for (int annotationId : group.members) {
    members.insert(idToAnnotation.at(annotationId));
  }
}

std::unordered_set<Annotation*> Group::getAll(const std::vector<Group*>& groups) {
  std::unordered_set<Annotation*> annotations;
  for (Group* group : groups) {
    std::copy(group->members.begin(), group->members.end(), std::inserter(annotations, annotations.end()));
  }
  return annotations;
}

std::unordered_set<Annotation*> Group::getCircles(const std::vector<Group*>& groups) {
  std::unordered_set<Annotation*> annotations;
  for (Group* group : groups) {
    for (Annotation* annotation : group->members) {
      if (annotation->info.type() == typeid(Annotation::Circle)) {
        annotations.insert(annotation);
      }
    }
  }
  return annotations;
}

std::unordered_set<Annotation*> Group::getLines(const std::vector<Group*>& groups) {
  std::unordered_set<Annotation*> annotations;
  for (Group* group : groups) {
    for (Annotation* annotation : group->members) {
      if (annotation->info.type() == typeid(Annotation::Polygon) &&
          boost::get<Annotation::Polygon>(annotation->info).points.size() == 2) {
        annotations.insert(annotation);
      }
    }
  }
  return annotations;
}

std::unordered_set<Annotation*> Group::getPolygons(const std::vector<Group*>& groups) {
  std::unordered_set<Annotation*> annotations;
  for (Group* group : groups) {
    for (Annotation* annotation : group->members) {
      if (annotation->info.type() == typeid(Annotation::Polygon) &&
          boost::get<Annotation::Polygon>(annotation->info).points.size() > 2) {
        annotations.insert(annotation);
      }
    }
  }
  return annotations;
}

std::unordered_multimap<DataFrame*, Annotation*> Group::getAnnotationsByFrame() const {
  std::unordered_multimap<DataFrame*, Annotation*> output;
  for (auto i = members.begin(); i != members.end(); ++i) {
    Annotation* member = *i;
    output.emplace(member->frame, member);
  }
  return output;
}

StreamableGroup::StreamableGroup(int id, const Group& group, const std::unordered_map<Annotation*, int>& annotationToId)
    : id(id), name(group.name), color(group.color.name().toUtf8().constData()) {
  for (const auto& annotation : group.members) {
    members.push_back(annotationToId.at(annotation));
  }
}

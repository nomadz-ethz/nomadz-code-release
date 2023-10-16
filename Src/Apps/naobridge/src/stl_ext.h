/**
 * @file stl_ext.h
 *
 * This file is subject to the terms of the HTWK License.
 * A copy of this license is included in LICENSE.HTWK.txt.
 * (c) 2023 HTWK and NomadZ team
 */

#pragma once

#include <chrono>
#include <cmath>
#include <experimental/optional>
#include <type_traits>
#include <vector>

template <typename Map>
std::experimental::optional<const typename Map::mapped_type> find(const Map& map, const typename Map::key_type& key) {
  auto it = map.find(key);
  if (it != map.end()) {
    return it->second;
  }
  return {};
}

template <typename C1, typename C2, typename F> void combined_for_each(C1&& c1, C2&& c2, F f) {
  auto it = std::cbegin(c2);
  for (const auto& e1 : c1) {
    f(e1, *it);
    it++;
  }
}

template <typename C1, typename C2, typename F> void zip(C1&& c1, C2&& c2, F f) { // alias for combined_for_each
  combined_for_each(std::forward<C1>(c1), std::forward<C2>(c2), std::forward<F>(f));
}

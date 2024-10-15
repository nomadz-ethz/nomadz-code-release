#pragma once

#include <array>

#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_behavior/data_types/ego_status.hpp"

namespace nomadz_behavior {
  constexpr int MAX_NUM_OF_PLAYERS = 7;

  struct TeammatesStatus {
    enum class Role { Goalkeeper, Player };
    std::array<EgoStatus, MAX_NUM_OF_PLAYERS> teammate_ego_status;
    std::array<Role, MAX_NUM_OF_PLAYERS> role;
    std::array<bool, MAX_NUM_OF_PLAYERS> is_active;
  };

} // namespace nomadz_behavior

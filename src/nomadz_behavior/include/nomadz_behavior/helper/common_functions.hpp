#pragma once

#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_behavior/data_types/teammates_status.hpp"

namespace nomadz_behavior {

  Eigen::Vector2f globalFromLocal(const Eigen::Vector2f& position, const nomadz_core::Pose2D& robot_pose);

  int getNumberHigherActiveFieldPlayers(int index, const std::array<bool, MAX_NUM_OF_PLAYERS>& active_status);
  int getNumberLowerActiveFieldPlayers(int index, const std::array<bool, MAX_NUM_OF_PLAYERS>& active_status);

} // end namespace nomadz_behavior

#include "nomadz_behavior/helper/common_functions.hpp"

namespace nomadz_behavior {

  Eigen::Vector2f globalFromLocal(const Eigen::Vector2f& position, const nomadz_core::Pose2D& robot_pose) {

    return nomadz_core::toAffine2(robot_pose) * position;
  }

  int getNumberHigherActiveFieldPlayers(const int index, const std::array<bool, MAX_NUM_OF_PLAYERS>& active_status) {
    int number_active_players = 0;

    for (int i = index + 1; i < MAX_NUM_OF_PLAYERS; i++) {
      if (active_status.at(i)) {
        number_active_players += 1;
      }
    }

    return number_active_players;
  }

  int getNumberLowerActiveFieldPlayers(const int index, const std::array<bool, MAX_NUM_OF_PLAYERS>& active_status) {
    int number_active_players = 0;
    for (int i = index - 1; i >= 0; i--) {
      if (active_status.at(i)) {
        number_active_players += 1;
      }
    }

    return number_active_players;
  }

} // end namespace nomadz_behavior

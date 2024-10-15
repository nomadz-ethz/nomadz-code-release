#pragma once

#include <vector>
#include <array>
#include <Eigen/Core>

#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_behavior/data_types/teammates_status.hpp"

namespace nomadz_behavior {
  struct BallModel {
    Eigen::Vector2f position;
    Eigen::Vector2f velocity;
    bool valid;
    bool lost;
  };

  struct RobotPose {
    nomadz_core::Pose2D pose;
    bool valid;
    bool lost;
  };

  struct WorldModel {
    BallModel ball_model;
    RobotPose robot_pose;
    std::vector<nomadz_core::Pose2D> teammate_poses;
    std::vector<nomadz_core::Pose2D> opponent_poses;
  };

  struct CombinedWorldModel {
    BallModel team_ball_model;
    std::array<nomadz_core::Pose2D, MAX_NUM_OF_PLAYERS> teammate_poses;
    std::array<nomadz_core::Pose2D, MAX_NUM_OF_PLAYERS> opponent_poses;
  };
} // namespace nomadz_behavior

#pragma once

#include <behaviortree_cpp/blackboard.h>

#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_behavior/helper/bt_enums.hpp"
#include "nomadz_behavior/data_types/modeling.hpp"
#include "nomadz_behavior/data_types/teammates_status.hpp"
#include "nomadz_behavior/data_types/game_status.hpp"
#include "nomadz_behavior/data_types/tuning_param/target_pose_generator_param.hpp"
#include "nomadz_configuration/game_settings.hpp"
#include "nomadz_configuration/field_dimensions.hpp"

namespace nomadz_behavior {
  class TargetPoseGenerator {
  private:
    using Pose2D = nomadz_core::Pose2D;
    using FieldLines = nomadz_configuration::FieldLines;
    using GameSettings = nomadz_configuration::GameSettings;

  public:
    explicit TargetPoseGenerator(BT::Blackboard::Ptr blackboard);

    bool getTargetPose(TargetType request, Pose2D& target_pose);

  private:
    Pose2D getReadyPose() const;
    Pose2D getOptimalCoverPose() const;
    Pose2D getObservationPose() const;
    Pose2D getEngagePose() const;
    Pose2D getOptimalSearchPose() const;
    Pose2D getCenterGoalPose() const;
    Pose2D getDetectShotPose() const;
    Pose2D getPenaltyKickPose() const;

    // helper functions optimization
    int getNumberActiveFieldPlayers(int index) const;
    Pose2D denormalizePose(const Pose2D& pose) const;
    Eigen::Vector2f getBallPosition() const;
    Pose2D getPoseDistanceToBall(float desired_distance) const;

    Pose2D getOptimalPose(bool ball_lost) const;
    float getCostSearch(const std::vector<float>& position) const;
    float getCostPlay(const std::vector<float>& position) const;

    float getCoverCost(const std::vector<float>& position) const;
    float getBallCoMCost(const std::vector<float>& position, const std::vector<float>& des_CoM) const;
    float getAvoidanceCost(const std::vector<float>& position) const;
    float getGoalFreeCost(const std::vector<float>& position) const;
    float getOffenseDefenseCost(const std::vector<float>& position) const;

    WorldModel world_model_;
    CombinedWorldModel combined_world_model_;
    nomadz_configuration::FieldLines field_lines_;
    TeammatesStatus teammates_status_;
    TargetPoseGeneratorParameters opt_variables_;

    int player_id_;

    BT::Blackboard::Ptr blackboard_;
  };

} // namespace nomadz_behavior

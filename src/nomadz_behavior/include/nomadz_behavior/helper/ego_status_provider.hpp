#pragma once

#include <limits>
#include <array>
#include <rclcpp/rclcpp.hpp>

#include "nomadz_behavior/data_types/teammates_status.hpp"
#include "nomadz_behavior/data_types/modeling.hpp"
#include "nomadz_configuration/game_settings.hpp"

#include "nomadz_communication_msgs/msg/team_comm_info.hpp"
#include "nomadz_modeling_msgs/msg/world_model.hpp"
#include "nomadz_behavior_msgs/msg/ball_lock_status.hpp"
#include "nomadz_communication_msgs/msg/robocup_game_control_data.hpp"

namespace nomadz_behavior {

  class EgoStatusProvider : public rclcpp::Node {
  private:
    enum class BallLockState { NO_LOCK, REQ_LOCK, HAS_LOCK };
    struct TeamMateBallLockInfo {
      rclcpp::Time last_valid_time_stamp_;
      int player_id{1};
      BallLockState ball_lock_state{BallLockState::NO_LOCK};

      float ball_score_when_valid{std::numeric_limits<float>::infinity()};
      float ball_score{std::numeric_limits<float>::infinity()};
    };

    using TeamCommInfoMsgT = nomadz_communication_msgs::msg::TeamCommInfo;
    using WorldModelMsgT = nomadz_modeling_msgs::msg::WorldModel;
    using GCDataMsgT = nomadz_communication_msgs::msg::RobocupGameControlData;

    using EgoStatusMsgT = nomadz_behavior_msgs::msg::EgoStatus;

    static constexpr const char* GC_DATA_TOPIC = "communication/gc_data";
    static constexpr const char* TEAM_COMM_INFO_TOPIC = "communication/team_comm_info";
    static constexpr const char* WORLD_MODEL_TOPIC = "modeling/world_model";

    static constexpr const char* EGO_STATUS_TOPIC = "behavior/ego_status";

    static constexpr float BALL_LOST_SCORE = 4.F;

  public:
    explicit EgoStatusProvider(const rclcpp::NodeOptions& options);

  private:
    float computeBallScore();

    float getTimeSince(const rclcpp::Time& time_stamp);

    void updateBallLock();
    void teamCommInfoCallback();

    BallModel ball_model_;
    RobotPose robot_pose_;
    TeamCommInfoMsgT team_comm_info_msg_;

    nomadz_configuration::GameSettings game_settings_;

    float ball_score_when_valid_{std::numeric_limits<float>::infinity()};
    rclcpp::Time last_valid_ball_time_stamp_;

    bool any_teammate_has_ball_lock_{false};
    int number_of_teammates_with_ball_lock_{0};
    float ball_score_from_has_lock_{std::numeric_limits<float>::infinity()};
    float lowest_teammate_ball_score_{std::numeric_limits<float>::infinity()};

    BallLockState ball_lock_state_{BallLockState::NO_LOCK};
    BallLockState last_ball_lock_state_{BallLockState::NO_LOCK};

    std::array<TeamMateBallLockInfo, MAX_NUM_OF_PLAYERS> teammates_ball_lock_info_;

    EgoStatusMsgT ego_status_msg_;

    rclcpp::Subscription<GCDataMsgT>::SharedPtr gc_data_sub_;
    rclcpp::Subscription<TeamCommInfoMsgT>::SharedPtr team_comm_info_sub_;
    rclcpp::Subscription<WorldModelMsgT>::SharedPtr world_model_sub_;

    rclcpp::Publisher<EgoStatusMsgT>::SharedPtr ego_status_pub_;
  };
} // namespace nomadz_behavior

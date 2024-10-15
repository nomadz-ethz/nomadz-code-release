#include "nomadz_behavior/helper/ego_status_provider.hpp"

#include "nomadz_configuration/io.hpp"
#include "nomadz_behavior/helper/ros_conversion.hpp"
#include "nomadz_communication_msgs/msg/team_comm_data.hpp"
#include "nomadz_communication_msgs/robocup_game_control_data_enums.hpp"

namespace nomadz_behavior {

  EgoStatusProvider::EgoStatusProvider(const rclcpp::NodeOptions& options) : Node("ego_status_provider", options) {

    for (auto& ball_lock_info : teammates_ball_lock_info_) {
      ball_lock_info.ball_score = std::numeric_limits<float>::infinity();
      ball_lock_info.ball_score_when_valid = std::numeric_limits<float>::infinity();
      ball_lock_info.last_valid_time_stamp_ = this->now();
    }
    last_valid_ball_time_stamp_ = this->now();

    game_settings_ = nomadz_configuration::getGameSettings();
    ego_status_pub_ = create_publisher<EgoStatusMsgT>(EGO_STATUS_TOPIC, 1);

    gc_data_sub_ = create_subscription<GCDataMsgT>(GC_DATA_TOPIC, 1, [this](GCDataMsgT::ConstSharedPtr msg) {
      for (const auto& team : msg->teams) {
        if (team.team_id == game_settings_.team_id) {
          ego_status_msg_.is_penalized = team.players[game_settings_.player_id - 1].penalty !=
                                         static_cast<uint8_t>(nomadz_communication_msgs::Penalty::NONE);
          break;
        }
      }
      ego_status_msg_.header.stamp = this->now();
      ego_status_pub_->publish(ego_status_msg_);
    });
    team_comm_info_sub_ =
      create_subscription<TeamCommInfoMsgT>(TEAM_COMM_INFO_TOPIC, 1, [this](TeamCommInfoMsgT::ConstSharedPtr msg) {
        team_comm_info_msg_ = *msg;
        teamCommInfoCallback();
        updateBallLock();
      });

    world_model_sub_ = create_subscription<WorldModelMsgT>(WORLD_MODEL_TOPIC, 1, [this](WorldModelMsgT::ConstSharedPtr msg) {
      ball_model_ = unpackBallModel(msg->ball_model);
      robot_pose_ = unpackRobotPose(msg->robot_pose);
      updateBallLock();
    });
  }

  void EgoStatusProvider::teamCommInfoCallback() {
    any_teammate_has_ball_lock_ = false;
    ball_score_from_has_lock_ = std::numeric_limits<float>::infinity();
    number_of_teammates_with_ball_lock_ = 0;
    for (int i = 0; i < MAX_NUM_OF_PLAYERS; i++) {
      if (i == game_settings_.player_id - 1) {
        continue;
      }
      nomadz_communication_msgs::msg::TeamCommData team_comm_data = team_comm_info_msg_.team_comm_info[i];
      if (team_comm_data.ego_status.ball_score != teammates_ball_lock_info_[i].ball_score ||
          team_comm_data.ego_status.is_penalized) {
        if (team_comm_data.ego_status.is_penalized) {
          teammates_ball_lock_info_[i].ball_score_when_valid = BALL_LOST_SCORE;
        } else {
          teammates_ball_lock_info_[i].ball_score_when_valid = team_comm_data.ego_status.ball_score;
          teammates_ball_lock_info_[i].last_valid_time_stamp_ = this->now();
        }
        teammates_ball_lock_info_[i].ball_score = teammates_ball_lock_info_[i].ball_score_when_valid;
      }

      lowest_teammate_ball_score_ = std::min(lowest_teammate_ball_score_, teammates_ball_lock_info_[i].ball_score);
      if (team_comm_data.ego_status.has_ball_lock) {
        teammates_ball_lock_info_[i].ball_lock_state = BallLockState::HAS_LOCK;
        any_teammate_has_ball_lock_ = true;
        ball_score_from_has_lock_ = std::min(ball_score_from_has_lock_, teammates_ball_lock_info_[i].ball_score);
        number_of_teammates_with_ball_lock_++;
      } else {
        teammates_ball_lock_info_[i].ball_lock_state = BallLockState::NO_LOCK;
      }
    }
  }

  float EgoStatusProvider::computeBallScore() {
    const float base_ball_score = ball_model_.position.norm();
    const float robot_lost_score_modifier = robot_pose_.lost ? 1.1F : 1.0F;
    const float keeper_modifier = (game_settings_.player_role == "keeper") ? 1.3F : 1.F;

    float local_ball_score = (base_ball_score * robot_lost_score_modifier * keeper_modifier);
    if (ball_model_.valid) {
      ball_score_when_valid_ = local_ball_score;
      last_valid_ball_time_stamp_ = this->now();
    } else if (!ball_model_.lost) {
      const float t = (getTimeSince(last_valid_ball_time_stamp_) - 0.3F) / (2.F - 0.3F);
      local_ball_score = BALL_LOST_SCORE * t + ball_score_when_valid_ * (1 - t);
    } else {
      local_ball_score = BALL_LOST_SCORE;
    }
    return local_ball_score;
  }

  void EgoStatusProvider::updateBallLock() {
    for (int i = 0; i < MAX_NUM_OF_PLAYERS; i++) {
      teammates_ball_lock_info_[i].ball_score = teammates_ball_lock_info_[i].ball_score_when_valid +
                                                getTimeSince(teammates_ball_lock_info_[i].last_valid_time_stamp_) / 4.F;
    }
    float ball_score = computeBallScore();
    bool want_ball = ball_score < lowest_teammate_ball_score_ && ball_model_.valid;
    switch (ball_lock_state_) {
    case BallLockState::NO_LOCK:
      if (number_of_teammates_with_ball_lock_ < 2 && want_ball) {
        ball_lock_state_ = BallLockState::HAS_LOCK;
      }
      break;
    case BallLockState::HAS_LOCK:
      if (ball_model_.lost || ball_score > lowest_teammate_ball_score_) {
        ball_lock_state_ = BallLockState::NO_LOCK;
      }
      break;
    default:
      ball_lock_state_ = BallLockState::NO_LOCK;
      break;
    }
    if (ball_lock_state_ != last_ball_lock_state_) {
      last_ball_lock_state_ = ball_lock_state_;

      ego_status_msg_.has_ball_lock = ball_lock_state_ == BallLockState::HAS_LOCK;
      ego_status_msg_.want_ball = want_ball;
      ego_status_msg_.ball_score = ball_score;
      ego_status_msg_.header.stamp = this->now();
      ego_status_pub_->publish(ego_status_msg_);
    }
  }

  float EgoStatusProvider::getTimeSince(const rclcpp::Time& time_stamp) {
    return static_cast<float>((static_cast<rclcpp::Duration>(this->now() - time_stamp)).seconds());
  }

} // namespace nomadz_behavior

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nomadz_behavior::EgoStatusProvider)

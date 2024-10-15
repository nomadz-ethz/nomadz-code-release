#pragma once

#include <rclcpp/rclcpp.hpp>
#include <boost/circular_buffer.hpp>

#include "nomadz_modeling/ball_locator.hpp"
#include "nomadz_modeling/self_locator.hpp"
#include "self_world_model_params.hpp"

#include "nomadz_motion_control_msgs/msg/motion_request.hpp"
#include "nomadz_behavior_msgs/msg/game_status.hpp"
#include "nomadz_behavior_msgs/msg/ego_status.hpp"
#include "nomadz_communication_msgs/robocup_game_control_data_enums.hpp"
#include "nomadz_modeling_msgs/msg/ball_model.hpp"
#include "nomadz_modeling_msgs/msg/robot_pose_list.hpp"
#include "nomadz_modeling_msgs/msg/world_model.hpp"
#include "nomadz_modeling_msgs/msg/registered_lines.hpp"
#include "nomadz_motion_control_msgs/msg/motion_info.hpp"
#include "nomadz_vision_msgs/msg/field_perception_data.hpp"
#include "nomadz_vision_msgs/msg/ball.hpp"

namespace nomadz_modeling {

  class WorldModelNode : public rclcpp::Node {
    using WorldModelMsgT = nomadz_modeling_msgs::msg::WorldModel;
    using RobotPoseListMsgT = nomadz_modeling_msgs::msg::RobotPoseList;
    using RegisteredLinesMsgT = nomadz_modeling_msgs::msg::RegisteredLines;
    using MotionRequestMsgT = nomadz_motion_control_msgs::msg::MotionRequest;
    using MotionInfoMsgT = nomadz_motion_control_msgs::msg::MotionInfo;
    using FieldPerceptionDataMsgT = nomadz_vision_msgs::msg::FieldPerceptionData;
    using BallMsgT = nomadz_vision_msgs::msg::Ball;
    using EgoStatusMsgT = nomadz_behavior_msgs::msg::EgoStatus;
    using GameStatusMsgT = nomadz_behavior_msgs::msg::GameStatus;

  public:
    explicit WorldModelNode(const rclcpp::NodeOptions& options);

  private:
    static constexpr float BALL_NOT_SEEN_TIMEOUT_S = 2.0;

    void setMotionSafety(MotionRequestMsgT::ConstSharedPtr msg);
    void locatorMotionUpdate(MotionInfoMsgT::ConstSharedPtr msg);
    void ballUpdate(BallMsgT::ConstSharedPtr msg);
    void locatorSensorUpdate(FieldPerceptionDataMsgT::ConstSharedPtr msg);
    void checkPenalizedOrFallen(EgoStatusMsgT::ConstSharedPtr msg);
    void checkWalkInGameState(GameStatusMsgT::ConstSharedPtr msg);

    void worldModelCallback();
    void publishRegisteredLandmarks(std::vector<RegisteredLine> registered_lines,
                                    std::vector<RegisteredLandmark> registered_landmarks);

    double getTimeSince(const rclcpp::Time& time_stamp) const {
      return static_cast<double>((static_cast<rclcpp::Duration>(this->now() - time_stamp)).seconds());
    }

    bool robotPoseIsValid() const { return getTimeSince(last_sensor_update_time_) < parameters_.valid_timeout; }

    bool robotPoseIsLost() const;

    self_world_model_params::Params parameters_;

    SelfLocator self_locator_;
    BallLocator ball_locator_;

    rclcpp::Time last_sensor_update_time_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<WorldModelMsgT>::SharedPtr world_model_pub_;
    rclcpp::Publisher<RobotPoseListMsgT>::SharedPtr debug_ukf_state_pub_;
    rclcpp::Publisher<RegisteredLinesMsgT>::SharedPtr registered_lines_pub_;

    rclcpp::Subscription<MotionRequestMsgT>::SharedPtr motion_request_sub_;
    rclcpp::Subscription<MotionInfoMsgT>::SharedPtr motion_info_sub_;
    rclcpp::Subscription<FieldPerceptionDataMsgT>::SharedPtr field_perception_sub_;

    rclcpp::Time last_ball_seen_time_;
    rclcpp::Subscription<BallMsgT>::SharedPtr ball_sub_;
    rclcpp::Subscription<EgoStatusMsgT>::SharedPtr ego_status_sub_;
    rclcpp::Subscription<GameStatusMsgT>::SharedPtr game_status_sub_;

    nomadz_communication_msgs::GameState last_game_state_{nomadz_communication_msgs::GameState::INITIAL};
    boost::circular_buffer<bool> is_moving_{80};
    bool is_motion_safe_ = true;
  };

} // namespace nomadz_modeling

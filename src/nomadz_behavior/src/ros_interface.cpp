#include "nomadz_behavior/ros_interface.hpp"

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>

#include "nomadz_behavior/helper/ros_conversion.hpp"
#include "nomadz_behavior/data_types/modeling.hpp"
#include "nomadz_behavior/data_types/teammates_status.hpp"
#include "nomadz_behavior/data_types/game_status.hpp"
#include "nomadz_communication_msgs/msg/team_info.hpp"

namespace nomadz_behavior {

  RosInterfaceNode::RosInterfaceNode(const std::string& node_name,
                                     const rclcpp::NodeOptions& options,
                                     nomadz_configuration::GameSettings game_settings,
                                     BT::Blackboard::Ptr blackboard)
      : Node(node_name, options), game_settings_(std::move(game_settings)), blackboard_(std::move(blackboard)) {
    initSubscribers();
    initPublishers();
    initServices();
    initBlackboard();
  };

  void RosInterfaceNode::initBlackboard() {
    blackboard_->set<GameStatus>("game_status", GameStatus());
    blackboard_->set<WorldModel>("world_model", WorldModel());
    blackboard_->set<CombinedWorldModel>("combined_world_model", CombinedWorldModel());
    blackboard_->set<RobotPosture>("robot_posture", RobotPosture());
    blackboard_->set<TeammatesStatus>("teammates_status", TeammatesStatus());
    blackboard_->set<MotionInfo>("motion_info", MotionInfo());
    blackboard_->set("is_motion_done", false);
    blackboard_->set("is_head_motion_done", false);

    blackboard_->set("ego_status", EgoStatus());
    blackboard_->set("motion_request", MotionRequest());
    blackboard_->set("head_motion_request", HeadMotionRequest());
  }

  void RosInterfaceNode::initSubscribers() {
    gc_data_sub_ = create_subscription<RobocupGameControlDataMsgT>(
      GC_DATA_TOPIC, 1, [this](RobocupGameControlDataMsgT::ConstSharedPtr msg) { gc_data_msg_ = *msg; });

    team_comm_info_sub_ = create_subscription<TeamCommInfoMsgT>(
      TEAM_COMM_INFO_TOPIC, 1, [this](TeamCommInfoMsgT::ConstSharedPtr msg) { team_comm_info_msg_ = *msg; });

    foot_support_sub_ = create_subscription<FootSupportMsgT>(
      FOOT_SUPPORT_TOPIC, 1, [this](FootSupportMsgT::ConstSharedPtr msg) { foot_support_msg_ = *msg; });

    fall_down_state_sub_ = create_subscription<FallDownStateMsgT>(
      FALL_DOWN_STATE_TOPIC, 1, [this](FallDownStateMsgT::ConstSharedPtr msg) { fall_down_state_msg_ = *msg; });

    motion_info_sub_ = create_subscription<MotionInfoMsgT>(MOTION_INFO_TOPIC, 1, [this](MotionInfoMsgT::ConstSharedPtr msg) {
      const MotionRequest motion_request = blackboard_->get<MotionRequest>("motion_request");
      const MotionInfo motion_info = unpackMotionInfo(*msg);
      const MotionRequest executed_motion_request = motion_info.executed_motion_request;
      bool is_requested_motion = false;
      if (motion_request.motion_type != static_cast<MotionType>(msg->executed_motion_request.motion_type)) {
        is_requested_motion = false;
      } else {
        if (motion_request.motion_type == MotionType::SPECIAL_ACTION) {
          is_requested_motion = motion_request.special_action_request.special_action_type ==
                                executed_motion_request.special_action_request.special_action_type;
        } else {
          is_requested_motion = true;
        }
      }

      const HeadMotionRequest head_motion_request = blackboard_->get<HeadMotionRequest>("head_motion_request");
      const HeadMotionRequest executed_head_motion_request = motion_info.executed_head_motion_request;

      bool is_requested_head_motion = false;
      if (head_motion_request.head_motion_type != executed_head_motion_request.head_motion_type) {
        is_requested_head_motion = false;
      } else {
        if (head_motion_request.head_motion_type == HeadMotionType::DIRECT) {
          is_requested_head_motion = head_motion_request.pan == executed_head_motion_request.pan &&
                                     head_motion_request.tilt == executed_head_motion_request.tilt;
        } else {
          is_requested_head_motion = head_motion_request.target == executed_head_motion_request.target;
        }
      }
      blackboard_->set<MotionInfo>("motion_info", motion_info);
      blackboard_->set("is_motion_done", motion_info.is_motion_done && is_requested_motion);
      blackboard_->set("is_head_motion_done", motion_info.is_head_motion_done && is_requested_head_motion);
    });

    ego_status_sub_ = create_subscription<EgoStatusMsgT>(EGO_STATUS_TOPIC, 1, [this](EgoStatusMsgT::ConstSharedPtr msg) {
      blackboard_->set<EgoStatus>("ego_status", unpackEgoStatus(*msg));
    });

    game_status_sub_ = create_subscription<GameStatusMsgT>(GAME_STATUS_TOPIC, 1, [this](GameStatusMsgT::ConstSharedPtr msg) {
      blackboard_->set<GameStatus>("game_status", unpackGameStatus(*msg));
    });

    world_model_sub_ = create_subscription<WorldModelMsgT>(WORLD_MODEL_TOPIC, 1, [this](WorldModelMsgT::ConstSharedPtr msg) {
      blackboard_->set<WorldModel>("world_model", unpackWorldModel(*msg));
      blackboard_->set<RobotPosture>("robot_posture", unpackRobotPosture(fall_down_state_msg_, foot_support_msg_));
      blackboard_->set<CombinedWorldModel>(
        "combined_world_model", unpackCombinedWorldModel(*msg, team_comm_info_msg_, this->now(), game_settings_.player_id));
      nomadz_communication_msgs::msg::TeamInfo own_team_info_msg;
      for (const auto& team : gc_data_msg_.teams) {
        if (team.team_id == game_settings_.team_id) {
          own_team_info_msg = team;
          break;
        }
      }
      blackboard_->set<TeammatesStatus>("teammates_status", unpackTeammateStatus(team_comm_info_msg_, own_team_info_msg));
    });
  };

  void RosInterfaceNode::initPublishers() {
    motion_request_pub_ = create_publisher<MotionRequestMsgT>(MOTION_REQUEST_TOPIC, 1);
  };

  void RosInterfaceNode::initServices(){};

  void RosInterfaceNode::publish() {
    publish(blackboard_->get<MotionRequest>("motion_request"), blackboard_->get<HeadMotionRequest>("head_motion_request"));
  }

  void RosInterfaceNode::publish(const MotionRequest& motion_request, const HeadMotionRequest& head_motion_request) {
    motion_request_pub_->publish(packMotionRequest(motion_request, head_motion_request, this->now()));
  }

} // namespace nomadz_behavior

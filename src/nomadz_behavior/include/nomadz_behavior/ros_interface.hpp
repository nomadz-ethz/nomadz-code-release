#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>

#include "nomadz_behavior_msgs/msg/ego_status.hpp"
#include "nomadz_behavior_msgs/msg/game_status.hpp"
#include "nomadz_motion_control_msgs/msg/motion_request.hpp"
#include "nomadz_motion_control_msgs/msg/walk_request.hpp"
#include "nomadz_motion_control_msgs/msg/kick_request.hpp"
#include "nomadz_motion_control_msgs/msg/special_action_request.hpp"
#include "nomadz_motion_control_msgs/msg/head_motion_request.hpp"
#include "nomadz_modeling_msgs/msg/world_model.hpp"
#include "nomadz_modeling_msgs/msg/ball_model.hpp"
#include "nomadz_modeling_msgs/msg/robot_pose.hpp"
#include "nomadz_communication_msgs/msg/robocup_game_control_data.hpp"
#include "nomadz_communication_msgs/msg/team_info.hpp"
#include "nomadz_communication_msgs/msg/team_comm_info.hpp"
#include "nomadz_proprioception_msgs/msg/fall_down_state.hpp"
#include "nomadz_proprioception_msgs/msg/foot_support.hpp"
#include "nomadz_configuration/game_settings.hpp"
#include "nomadz_motion_control_msgs/msg/motion_info.hpp"
#include "nomadz_behavior/data_types/ego_status.hpp"
#include "nomadz_behavior/data_types/motion.hpp"
#include "nomadz_behavior/data_types/proprioception.hpp"

namespace nomadz_behavior {

  class RosInterfaceNode : public rclcpp::Node {
  private:
    using RobocupGameControlDataMsgT = nomadz_communication_msgs::msg::RobocupGameControlData;
    using TeamCommInfoMsgT = nomadz_communication_msgs::msg::TeamCommInfo;
    using FootSupportMsgT = nomadz_proprioception_msgs::msg::FootSupport;
    using FallDownStateMsgT = nomadz_proprioception_msgs::msg::FallDownState;
    using MotionInfoMsgT = nomadz_motion_control_msgs::msg::MotionInfo;
    using WorldModelMsgT = nomadz_modeling_msgs::msg::WorldModel;
    using EgoStatusMsgT = nomadz_behavior_msgs::msg::EgoStatus;
    using GameStatusMsgT = nomadz_behavior_msgs::msg::GameStatus;

    using MotionRequestMsgT = nomadz_motion_control_msgs::msg::MotionRequest;

    static constexpr const char* GC_DATA_TOPIC = "communication/gc_data";
    static constexpr const char* TEAM_COMM_INFO_TOPIC = "communication/team_comm_info";
    static constexpr const char* FOOT_SUPPORT_TOPIC = "proprioception/foot_support";
    static constexpr const char* FALL_DOWN_STATE_TOPIC = "proprioception/fall_down_state";
    static constexpr const char* MOTION_INFO_TOPIC = "motion_control/motion_info";
    static constexpr const char* WORLD_MODEL_TOPIC = "modeling/world_model";
    static constexpr const char* EGO_STATUS_TOPIC = "behavior/ego_status";
    static constexpr const char* GAME_STATUS_TOPIC = "behavior/game_status";

    static constexpr const char* MOTION_REQUEST_TOPIC = "behavior/motion_request";

  public:
    RosInterfaceNode(const std::string& node_name,
                     const rclcpp::NodeOptions& options,
                     nomadz_configuration::GameSettings game_settings,
                     BT::Blackboard::Ptr blackboard);
    void publish();

  private:
    void initBlackboard();
    void initSubscribers();
    void initPublishers();
    void initServices();

    void publish(const MotionRequest& motion_request, const HeadMotionRequest& head_motion_request);

    nomadz_configuration::GameSettings game_settings_;
    RobocupGameControlDataMsgT gc_data_msg_;
    TeamCommInfoMsgT team_comm_info_msg_;
    FootSupportMsgT foot_support_msg_;
    FallDownStateMsgT fall_down_state_msg_;

    std::shared_ptr<rclcpp::TimerBase> timer_;

    rclcpp::Subscription<RobocupGameControlDataMsgT>::SharedPtr gc_data_sub_;
    rclcpp::Subscription<TeamCommInfoMsgT>::SharedPtr team_comm_info_sub_;
    rclcpp::Subscription<FootSupportMsgT>::SharedPtr foot_support_sub_;
    rclcpp::Subscription<FallDownStateMsgT>::SharedPtr fall_down_state_sub_;
    rclcpp::Subscription<MotionInfoMsgT>::SharedPtr motion_info_sub_;
    rclcpp::Subscription<WorldModelMsgT>::SharedPtr world_model_sub_;
    rclcpp::Subscription<EgoStatusMsgT>::SharedPtr ego_status_sub_;
    rclcpp::Subscription<GameStatusMsgT>::SharedPtr game_status_sub_;

    rclcpp::Publisher<MotionRequestMsgT>::SharedPtr motion_request_pub_;

    BT::Blackboard::Ptr blackboard_;
  };
} // namespace nomadz_behavior

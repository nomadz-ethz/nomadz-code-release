#pragma once

#include <string>
#include <map>

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"

#include "nomadz_kinematics/robot_model.hpp"
#include "nomadz_motion_control/interpolation.hpp"
#include "nao_lola_sensor_msgs/msg/imu.hpp"
#include "nao_lola_sensor_msgs/msg/joint_data.hpp"
#include "nao_lola_command_msgs/msg/joint_requests.hpp"
#include "nomadz_motion_control_msgs/msg/motion_request.hpp"
#include "nomadz_proprioception_msgs/msg/robot_model.hpp"
#include "nomadz_proprioception_msgs/msg/fall_down_state.hpp"
#include "nomadz_proprioception_msgs/msg/arm_contact_model.hpp"
#include "nomadz_proprioception_msgs/msg/foot_support.hpp"
#include "nomadz_modeling_msgs/msg/world_model.hpp"

#include "nomadz_motion_control/motion_base.hpp"
#include "nomadz_motion_control/joint_requests.hpp"
#include "nomadz_motion_control_msgs/msg/motion_info.hpp"
#include "nomadz_motion_control_msgs/motion_request_enums.hpp"

namespace nomadz_motion_control {
  /**
   * @brief Main motion control node
   *
   * This class is intended to represent the bulk of the motion control. In particular,
   * it will hold pointer to the relevant controllers and perform control allocation
   * depending on the provided motion requests and current robot state.
   */
  class MotionControlNode : public rclcpp::Node {
    using JointRequestMsgT = nao_lola_command_msgs::msg::JointRequests;
    using MotionInfoMsgT = nomadz_motion_control_msgs::msg::MotionInfo;
    using MotionRequestMsgT = nomadz_motion_control_msgs::msg::MotionRequest;
    using FallDownStateMsgT = nomadz_proprioception_msgs::msg::FallDownState;
    using RobotModelMsgT = nomadz_proprioception_msgs::msg::RobotModel;
    using MotionType = nomadz_motion_control_msgs::MotionType;

  public:
    explicit MotionControlNode(const rclcpp::NodeOptions& options);
    MotionControlNode(const std::string& node_name, const rclcpp::NodeOptions& options);

  private:
    /**
     * @brief Resolves the controller
     *
     * Initializes the available controllers and inserts them into the
     * `motion_controller_map_`.
     */
    void loadController(const rclcpp::NodeOptions& options);

    /**
     * @brief Check if the previous controller has finished. If yes then update the active controller.
     */
    void updateController();

    /**
     * @brief Process falldown state message and take appropriate measures.
     *
     * Specifically, if a falldown is detected, the active controller is switched to the special action
     * controller and the requested motion is set to the appropriate falldown protection message.
     */
    void processFalldownState(FallDownStateMsgT::ConstSharedPtr msg);

    /**
     * @brief Override the MotionRequest received from the behavior.
     *
     * Overrides the incoming motion request from the behavior and executes the specified motion request
     * instead. If this funciton is called, the next motion request callback is blocked until the current
     * controller can leave.
     */
    void overrideMotionRequest(const MotionRequestMsgT& motion_request);

    void motionControlCallback();

    std::shared_ptr<MotionBase> arm_motion_engine_;
    std::shared_ptr<MotionBase> head_motion_engine_;

    // This map contains all the loaded controllers
    std::map<MotionType, std::shared_ptr<MotionBase>> motion_controller_map_;
    // Which controller is currently active and controlling the joints. Uses the constants
    // from the MotionRequest.msg to avoid unnecessary conversions.
    MotionType active_controller_;

    MotionType previous_controller_;
    JointRequestInterpolator interpolator_;
    nomadz_kinematics::RobotModel requested_robot_model_;
    MotionRequestMsgT current_motion_request_msg_; // The current executed motion request
    MotionRequestMsgT cached_motion_request_;      // The received motion request are stored here
    bool requested_controller_switch_{false};
    bool has_motion_request_been_overwritten_{false};
    bool unstiffed_{true};

    std::shared_ptr<rclcpp::TimerBase> timer_;
    bool is_on_ground_{false};

    rclcpp::executors::SingleThreadedExecutor executor_;

    rclcpp::Publisher<JointRequestMsgT>::SharedPtr joint_requests_pub_;
    rclcpp::Publisher<MotionInfoMsgT>::SharedPtr motion_info_pub_;
    rclcpp::Publisher<RobotModelMsgT>::SharedPtr requested_robot_model_pub_;

    rclcpp::Subscription<MotionRequestMsgT>::SharedPtr motion_request_sub_;
    rclcpp::Subscription<FallDownStateMsgT>::SharedPtr fall_down_state_sub_;
    rclcpp::Subscription<RobotModelMsgT>::SharedPtr requested_robot_model_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr unstiff_state_sub_;
  };
} // namespace nomadz_motion_control

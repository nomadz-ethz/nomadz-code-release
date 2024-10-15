#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>

#include "nao_lola_sensor_msgs/msg/joint_data.hpp"
#include "nao_lola_command_msgs/msg/joint_requests.hpp"
#include "nomadz_motion_control_msgs/msg/motion_request.hpp"
#include "nomadz_motion_control/joint_requests.hpp"
#include "nomadz_motion_control_msgs/msg/motion_info.hpp"
#include "nomadz_motion_control_msgs/motion_request_enums.hpp"

namespace nomadz_motion_control {
  /**
   * @brief Abstract base class for motion controllers
   *
   * This class declares the interface for the motion controllers used in
   * the `MotionControlNode`. Inheriting classes are expected to implement the
   * following functions:
   * - `updateJointRequests` which should compute new joint requests for the robot.
   * - `updateMotionInfoMsg` which should provide the motion info of the current motion.
   * - `reset` which resets the controller to a default configuration.
   * - `isLeavingPossible` which indicates that the motion of the controller can be left savely
   *   (either it is in a safe state or the action has completed).
   *
   * `updateJointRequests` and `updateMotionInfoMsg` are both called from the
   * `executeControlLoop` method, which is itself called in a timer callback by the
   * `MotionControlNode`.
   */
  class MotionBase : public rclcpp::Node {
  protected:
    using JointDataMsgT = nao_lola_sensor_msgs::msg::JointData;
    using MotionRequestMsgT = nomadz_motion_control_msgs::msg::MotionRequest;
    using MotionInfoMsgT = nomadz_motion_control_msgs::msg::MotionInfo;
    using SpecialActionRequestMsgT = nomadz_motion_control_msgs::msg::SpecialActionRequest;
    using WalkRequestMsgT = nomadz_motion_control_msgs::msg::WalkRequest;
    using KickRequestMsgT = nomadz_motion_control_msgs::msg::KickRequest;
    using MotionType = nomadz_motion_control_msgs::MotionType;

  public:
    /**
     * @brief constructor for the controller
     */
    MotionBase(const std::string& node_name, const rclcpp::NodeOptions& options);

    JointRequests getJointRequest() const;

    MotionInfoMsgT getMotionInfo() const;

    /**
     * @brief Executes the main control loop of the controller.
     */
    void executeControlLoop();

    /**
     * @brief Executes the main control loop of the controller.
     * Override the default subscribed motion request with the provided one.
     */
    void executeControlLoop(const nomadz_motion_control_msgs::msg::MotionRequest& motion_request);

    /**
     * @brief reset the controller
     *
     * This function is called after every controller switch to initialize
     * your controller.
     */
    virtual void reset() = 0;

    /**
     * @brief Whether leaving the current action is possible
     */
    virtual bool isLeavingPossible() = 0;

    /**
     * @brief Request to leave the current action
     */
    void requestLeave() { request_leave_ = true; };

  protected:
    /**
     * @brief Compute new joint requests
     *
     * This method is called in `executeControlLoop` and should be responsible
     * for computing new joint requests according to the motion request and the
     * current robot state.
     *
     * This function needs to be implemented in inheriting classes with the respective logic.
     */
    virtual void updateJointRequests() = 0;

    /**
     * @brief Update motion info of the controller
     *
     * This method should provide the motion info message for the current control
     * iteration.
     *
     * This function needs to be implemented in inheriting classes with the respective logic.
     */
    virtual void updateMotionInfoMsg() = 0;

    // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
    bool request_leave_{false};
    JointData current_joint_data_;
    JointRequests last_joint_requests_;
    JointRequests current_joint_requests_;

    MotionRequestMsgT current_motion_request_msg_;
    MotionInfoMsgT motion_info_msg_;
    // NOLINTEND(misc-non-private-member-variables-in-classes)

  private:
    rclcpp::Subscription<JointDataMsgT>::SharedPtr joint_data_sub_;
    rclcpp::Subscription<MotionRequestMsgT>::SharedPtr motion_request_sub_;
  };

} // namespace nomadz_motion_control

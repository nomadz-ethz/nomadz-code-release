#pragma once

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include "nomadz_core/geometry/pose.hpp"
#include "nao_lola_sensor_msgs/msg/imu.hpp"
#include "nomadz_modeling_msgs/msg/world_model.hpp"
#include "nomadz_proprioception_msgs/msg/foot_support.hpp"
#include "nomadz_proprioception_msgs/msg/robot_model.hpp"
#include "nomadz_motion_control_msgs/msg/walk_request.hpp"
#include "nomadz_motion_control_msgs/msg/kick_request.hpp"

#include "nomadz_motion_control/motion_base.hpp"
#include "nomadz_motion_control/walk_engine/walk_core_state.hpp"
#include "nomadz_motion_control/walk_engine/step_planner.hpp"
#include "nomadz_motion_control/walk_engine/walk_generator.hpp"
#include "nomadz_motion_control/walk_engine/walk_generator_data.hpp"
#include "nomadz_motion_control/walk_engine/planned_steps.hpp"
#include "walk_engine_parameters.hpp"

namespace nomadz_motion_control {

  class WalkEngine : public MotionBase {
    using WorldModelMsgT = nomadz_modeling_msgs::msg::WorldModel;
    using FootSupportMsgT = nomadz_proprioception_msgs::msg::FootSupport;
    using RobotModelMsgT = nomadz_proprioception_msgs::msg::RobotModel;
    using ImuMsgT = nao_lola_sensor_msgs::msg::Imu;

    using WalkParamT = walk_engine_parameters::Params;
    using WalkMode = nomadz_motion_control_msgs::WalkMode;

  public:
    explicit WalkEngine(const rclcpp::NodeOptions& options);

  private:
    void computeNextStep();

    void preProcessWalkRequest(const WalkRequestMsgT& request);

    void preProcessKickRequest(const KickRequestMsgT& request);

    void checkSwitchConditions();

    void updateJointRequests() override;

    void updateMotionInfoMsg() override;

    bool isLeavingPossible() override;

    void reset() override;

    void stateCheck();

    static constexpr float SWITCH_THRESHOLD_MIN = 0.3F;
    static constexpr float SWITCH_THRESHOLD_MAX = 3.0F;

    walk_engine_parameters::ParamListener param_listener_;
    const std::shared_ptr<WalkParamT> dyn_params_;

    WalkCoreState walk_core_state_{};
    StepPlanner step_planner_;
    WalkGenerator walk_generator_;

    float walk_height_;
    bool force_zero_speed_{false};

    Eigen::Vector2f ball_position_;
    nomadz_proprioception_msgs::msg::FootSupport foot_support_;

    rclcpp::Subscription<ImuMsgT>::SharedPtr imu_sub_;
    rclcpp::Subscription<WorldModelMsgT>::SharedPtr world_model_sub_;
    rclcpp::Subscription<FootSupportMsgT>::SharedPtr foot_support_sub_;
    rclcpp::Subscription<RobotModelMsgT>::SharedPtr measured_robot_model_sub_;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr orientation_sub_;
  };
} // namespace nomadz_motion_control

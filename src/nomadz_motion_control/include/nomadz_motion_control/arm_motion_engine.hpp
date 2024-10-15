#pragma once

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include "nomadz_proprioception_msgs/msg/robot_model.hpp"
#include "nomadz_proprioception_msgs/msg/arm_contact_model.hpp"
#include "nomadz_motion_control/motion_base.hpp"
#include "nomadz_core/math/constants.hpp"
#include "arm_motion_engine_parameters.hpp"

namespace nomadz_motion_control {

  using EigenArr6f = Eigen::Array<float, 6, 1>;

  template <typename T> struct ArmPairT { T left, right; };

  enum class MotionMode { K_SWING, K_TO_BACK, K_DEFAULT };

  /**
   * @brief This instance generates the arm motion of the robot.
   */
  class ArmMotionEngine : public MotionBase {
    using RobotModelMsgT = nomadz_proprioception_msgs::msg::RobotModel;
    using ArmContactModelMsgT = nomadz_proprioception_msgs::msg::ArmContactModel;

  public:
    explicit ArmMotionEngine(const rclcpp::NodeOptions& options);

  private:
    void updateJointRequests() override;

    void updateMotionInfoMsg() override;

    bool isLeavingPossible() override;

    void reset() override;

    void initParameters();

    /**
     * @brief Updates the reference points for the swing motion.
     */
    void updateEndpoints();

    /**
     * @brief Based on predicates, determines the desired motion mode.
     */
    ArmPairT<MotionMode> computeDesiredMode();

    ArmPairT<EigenArr6f> computeSetpoints(ArmPairT<MotionMode> motion_modes);

    /**
     * @brief Computes the commanded joint point based on a simple P-controller.
     */
    EigenArr6f propagatePoint(MotionMode motion_mode, EigenArr6f current_point, EigenArr6f endpoint);

    /**
     * @brief Set a joint point for the opposite arm.
     */
    static EigenArr6f mirrorPoint(EigenArr6f joint_point);

    ArmPairT<EigenArr6f> getJointPoints();

    void setJointPoints(ArmPairT<EigenArr6f> joint_points);

    arm_motion_engine_parameters::ParamListener param_listener_;

    float swing_gain_, thrsh_shoulder_roll_, thrsh_shoulder_pitch_, shoulder_roll_offset_;
    rclcpp::Duration contact_delay_;
    EigenArr6f p_gains_;
    ArmPairT<rclcpp::Time> t_last_contact_;
    ArmPairT<Eigen::Array2f> foot_poses_;
    ArmPairT<std::map<MotionMode, EigenArr6f>> mode_endpoints_;

    const std::array<int, 6> kleft_idxs_ = {nomadz_definitions::joint_indexes::L_SHOULDER_ROLL,
                                            nomadz_definitions::joint_indexes::L_SHOULDER_PITCH,
                                            nomadz_definitions::joint_indexes::L_ELBOW_ROLL,
                                            nomadz_definitions::joint_indexes::L_ELBOW_YAW,
                                            nomadz_definitions::joint_indexes::L_WRIST_YAW,
                                            nomadz_definitions::joint_indexes::L_HAND};
    const std::array<int, 6> kright_idxs_ = {nomadz_definitions::joint_indexes::R_SHOULDER_ROLL,
                                             nomadz_definitions::joint_indexes::R_SHOULDER_PITCH,
                                             nomadz_definitions::joint_indexes::R_ELBOW_ROLL,
                                             nomadz_definitions::joint_indexes::R_ELBOW_YAW,
                                             nomadz_definitions::joint_indexes::R_WRIST_YAW,
                                             nomadz_definitions::joint_indexes::R_HAND};

    rclcpp::Subscription<RobotModelMsgT>::SharedPtr requested_robot_model_sub_;
    rclcpp::Subscription<ArmContactModelMsgT>::SharedPtr arm_contact_sub_;
  };

} // namespace nomadz_motion_control

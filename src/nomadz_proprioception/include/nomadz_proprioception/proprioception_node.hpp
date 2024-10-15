#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "nao_lola_sensor_msgs/msg/joint_data.hpp"
#include "nao_lola_sensor_msgs/msg/fsr.hpp"
#include "nao_lola_sensor_msgs/msg/imu.hpp"
#include "nomadz_kinematics/robot_model.hpp"
#include "nomadz_proprioception_msgs/msg/robot_model.hpp"
#include "nomadz_proprioception/foot_support_estimator.hpp"
#include "nomadz_proprioception/fall_down_state_detector.hpp"
#include "nomadz_proprioception/sensor_data_filter.hpp"
#include "nomadz_proprioception/arm_contact_estimator.hpp"

namespace nomadz_proprioception {

  class ProprioceptionNode : public rclcpp::Node {
  private:
    // use this conversion everywhere
    using JointDataMsgT = nao_lola_sensor_msgs::msg::JointData;
    using ImuMsgT = nao_lola_sensor_msgs::msg::Imu;
    using FsrMsgT = nao_lola_sensor_msgs::msg::Fsr;
    using RequestedRobotModelMsgT = nomadz_proprioception_msgs::msg::RobotModel;

    using FootSupportMsgT = nomadz_proprioception_msgs::msg::FootSupport;
    using FallDownStateMsgT = nomadz_proprioception_msgs::msg::FallDownState;
    using RobotModelMsgT = nomadz_proprioception_msgs::msg::RobotModel;
    using ArmContactModelMsgT = nomadz_proprioception_msgs::msg::ArmContactModel;

    static constexpr const char* IMU_TOPIC = "sensors/imu";
    static constexpr const char* JOINT_DATA_TOPIC = "sensors/joint_data";
    static constexpr const char* FSR_TOPIC = "sensors/fsr";
    static constexpr const char* REQUESTED_ROBOT_MODEL_TOPIC = "motion_control/requested_robot_model";

    static constexpr const char* FOOT_SUPPORT_TOPIC = "proprioception/foot_support";
    static constexpr const char* FALL_DOWN_STATE_TOPIC = "proprioception/fall_down_state";
    static constexpr const char* ORIENTATION_TOPIC = "proprioception/orientation";
    static constexpr const char* ARM_CONTACT_TOPIC = "proprioception/arm_contact";
    static constexpr const char* ROBOT_MODEL_TOPIC = "proprioception/measured_robot_model";

  public:
    explicit ProprioceptionNode(const rclcpp::NodeOptions& options);

  private:
    void setupStaticTransforms();

    void onSensorDataReceived(ImuMsgT::ConstSharedPtr imu_msg,
                              JointDataMsgT::ConstSharedPtr joint_data_msg,
                              FsrMsgT::ConstSharedPtr fsr_msg);

    void updateFootSupport(const nao_lola_sensor_msgs::msg::Fsr& fsr_msg);

    void updateFallDownState(const nao_lola_sensor_msgs::msg::Imu& imu_msg);

    void updateOrientation(const nao_lola_sensor_msgs::msg::Imu& msg);

    void updateTransforms(const nao_lola_sensor_msgs::msg::Imu& msg,
                          const nao_lola_sensor_msgs::msg::JointData& joint_data_msg);

    void updateRobotModel(const nao_lola_sensor_msgs::msg::JointData& joint_data_msg);
    void updateArmContactEstimate(const RobotModelMsgT& requested_robot_model_msg);

    FootSupportEstimator foot_support_estimator_;
    FallDownStateDetector fall_down_state_detector_;
    SensorDataFilter sensor_data_filter_;
    nomadz_kinematics::RobotModel measured_robot_model_;
    ArmContactEstimator arm_contact_estimator_;
    ArmContactModel arm_contact_model_;

    rclcpp::Publisher<FootSupportMsgT>::SharedPtr foot_support_publisher_;
    rclcpp::Publisher<FallDownStateMsgT>::SharedPtr fall_down_state_publisher_;
    rclcpp::Publisher<RobotModelMsgT>::SharedPtr robot_model_publisher_;
    rclcpp::Publisher<ArmContactModelMsgT>::SharedPtr arm_contact_publisher_;

    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr orientation_estimation_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;

    message_filters::Subscriber<ImuMsgT> imu_sub_;
    message_filters::Subscriber<JointDataMsgT> joint_data_sub_;
    message_filters::Subscriber<FsrMsgT> fsr_sub_;
    message_filters::TimeSynchronizer<ImuMsgT, JointDataMsgT, FsrMsgT> sync_;
    rclcpp::Subscription<RequestedRobotModelMsgT>::ConstSharedPtr requested_robot_model_sub_;
  };
} // namespace nomadz_proprioception

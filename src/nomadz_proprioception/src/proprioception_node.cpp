#include "nomadz_proprioception/proprioception_node.hpp"

#include "nomadz_definitions/joint_indexes.hpp"
#include "nomadz_definitions/limbs.hpp"
#include "nomadz_kinematics/forward_kinematics.hpp"
#include "nomadz_kinematics/ros_conversion.hpp"
#include "nomadz_core/geometry/ros_conversion.hpp"

#include "nomadz_proprioception/ros_conversion.hpp"
#include "nomadz_proprioception/transforms.hpp"

namespace limbs = nomadz_definitions::limbs;

using nomadz_core::unpackVector3;

namespace nomadz_proprioception {
  // clang-format off
  ProprioceptionNode::ProprioceptionNode(const rclcpp::NodeOptions& options)
      : Node("proprioception_node", options),
        imu_sub_(this, IMU_TOPIC),
        joint_data_sub_(this, JOINT_DATA_TOPIC),
        fsr_sub_(this, FSR_TOPIC),
        sync_(imu_sub_, joint_data_sub_, fsr_sub_, 5)
  // clang-format on
  {
    foot_support_publisher_ = create_publisher<FootSupportMsgT>(FOOT_SUPPORT_TOPIC, 1);
    fall_down_state_publisher_ = create_publisher<FallDownStateMsgT>(FALL_DOWN_STATE_TOPIC, 1);
    orientation_estimation_publisher_ = create_publisher<geometry_msgs::msg::QuaternionStamped>(ORIENTATION_TOPIC, 1);
    robot_model_publisher_ = create_publisher<RobotModelMsgT>(ROBOT_MODEL_TOPIC, 1);
    arm_contact_publisher_ = create_publisher<ArmContactModelMsgT>(ARM_CONTACT_TOPIC, 1);

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_transform_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    sync_.registerCallback(&ProprioceptionNode::onSensorDataReceived, this);
    requested_robot_model_sub_ = create_subscription<RequestedRobotModelMsgT>(
      REQUESTED_ROBOT_MODEL_TOPIC, 1, [this](RequestedRobotModelMsgT::ConstSharedPtr msg) {
        updateArmContactEstimate(*msg);
      });

    setupStaticTransforms();
  }

  void ProprioceptionNode::setupStaticTransforms() {
    RCLCPP_DEBUG(this->get_logger(), "Setting up static transforms");

    const Eigen::Affine3f transform_head_upper_camera = headFromCamera(Camera::UPPER_CAMERA);
    const Eigen::Affine3f transform_head_lower_camera = headFromCamera(Camera::LOWER_CAMERA);

    const rclcpp::Time stamp = this->now();
    static_transform_broadcaster_->sendTransform(
      packTransformStamped(transform_head_upper_camera, "head", "upper_camera", stamp));
    static_transform_broadcaster_->sendTransform(
      packTransformStamped(transform_head_lower_camera, "head", "lower_camera", stamp));
  }

  void ProprioceptionNode::onSensorDataReceived(ImuMsgT::ConstSharedPtr imu_msg,
                                                JointDataMsgT::ConstSharedPtr joint_data_msg,
                                                FsrMsgT::ConstSharedPtr fsr_msg) {
    assert(imu_msg && joint_data_msg && fsr_msg && "Received nullptr message");

    updateFootSupport(*fsr_msg);
    updateFallDownState(*imu_msg);
    updateOrientation(*imu_msg);
    updateTransforms(*imu_msg, *joint_data_msg);
    updateRobotModel(*joint_data_msg);
  }

  void ProprioceptionNode::updateArmContactEstimate(const RobotModelMsgT& requested_robot_model_msg) {
    arm_contact_estimator_.update(measured_robot_model_, requested_robot_model_msg);
    arm_contact_model_ = arm_contact_estimator_.getArmContactModel();
    ArmContactModelMsgT arm_contact_model_msg;
    arm_contact_model_msg.header.stamp = this->now();
    arm_contact_model_msg.contact_left =
      arm_contact_model_.contact_left && arm_contact_model_.duration_left > rclcpp::Duration::from_seconds(0.3);
    arm_contact_model_msg.contact_right =
      arm_contact_model_.contact_right && arm_contact_model_.duration_right > rclcpp::Duration::from_seconds(0.3);
    arm_contact_model_msg.duration_left = arm_contact_model_.duration_left;
    arm_contact_model_msg.duration_right = arm_contact_model_.duration_right;
    arm_contact_model_msg.push_direction_left = static_cast<uint8_t>(arm_contact_model_.push_direction_left);
    arm_contact_model_msg.push_direction_right = static_cast<uint8_t>(arm_contact_model_.push_direction_right);

    arm_contact_publisher_->publish(arm_contact_model_msg);
  }

  void ProprioceptionNode::updateFootSupport(const nao_lola_sensor_msgs::msg::Fsr& msg) {
    foot_support_estimator_.update(unpackFsr(msg), toTimestamp(msg.header.stamp));
    FootSupportMsgT foot_support_msg = foot_support_estimator_.getFootSupport();
    foot_support_msg.header.stamp = this->now();
    foot_support_publisher_->publish(foot_support_msg);
  }

  void ProprioceptionNode::updateFallDownState(const nao_lola_sensor_msgs::msg::Imu& msg) {
    fall_down_state_detector_.setAngle(msg.angle_roll, msg.angle_pitch);
    fall_down_state_detector_.updateGravityVectorAngle(msg.accelerometer);
    fall_down_state_detector_.updateFallDownState(foot_support_estimator_.getGroundContact());

    FallDownStateMsgT fall_down_state_msg;
    fall_down_state_msg.header.stamp = this->now();
    fall_down_state_msg.fall_down_state = static_cast<uint8_t>(fall_down_state_detector_.getFallDownState());
    fall_down_state_msg.fall_direction = static_cast<uint8_t>(fall_down_state_detector_.getFallDownDirection());
    fall_down_state_publisher_->publish(fall_down_state_msg);
  }

  void ProprioceptionNode::updateOrientation(const nao_lola_sensor_msgs::msg::Imu& msg) {

    RCLCPP_DEBUG(this->get_logger(), "Updating orientation with new IMU measurement");

    sensor_data_filter_.processGyroscope(unpackVector3(msg.gyroscope));
    sensor_data_filter_.processAccelerometer(unpackVector3(msg.accelerometer));

    geometry_msgs::msg::QuaternionStamped orientation_msg =
      packQuaternionStamped(sensor_data_filter_.orientation(), "torso", msg.header.stamp);
    orientation_estimation_publisher_->publish(orientation_msg);
  }

  void ProprioceptionNode::updateTransforms(const nao_lola_sensor_msgs::msg::Imu& imu_msg,
                                            const nao_lola_sensor_msgs::msg::JointData& joint_data_msg) {
    RCLCPP_DEBUG(this->get_logger(), "Updating transforms");

    std::array<Eigen::Affine3f, limbs::NUM_LIMBS> limb_transforms;
    nomadz_kinematics::calculateArmChain(true, joint_data_msg.positions, limb_transforms);
    nomadz_kinematics::calculateArmChain(false, joint_data_msg.positions, limb_transforms);
    nomadz_kinematics::calculateLegChain(true, joint_data_msg.positions, limb_transforms);
    nomadz_kinematics::calculateLegChain(false, joint_data_msg.positions, limb_transforms);
    nomadz_kinematics::calculateHeadChain(joint_data_msg.positions, limb_transforms);
    limb_transforms[limbs::TORSO] = groundFromTorso(imu_msg.angle_roll, imu_msg.angle_pitch, joint_data_msg.positions);

    std::vector<geometry_msgs::msg::TransformStamped> transform_msgs;
    transform_msgs.reserve(limbs::NUM_LIMBS);
    for (int i = 0; i < limbs::TORSO; ++i) {
      transform_msgs.push_back(packTransformStamped(limb_transforms[i],
                                                    limbs::getLimbName(limbs::getLimbParent(static_cast<limbs::Limbs>(i))),
                                                    limbs::getLimbName(static_cast<limbs::Limbs>(i)),
                                                    joint_data_msg.header.stamp));
    }

    transform_msgs.push_back(packTransformStamped(
      limb_transforms[limbs::TORSO], "ground", limbs::getLimbName(limbs::TORSO), joint_data_msg.header.stamp));

    transform_broadcaster_->sendTransform(transform_msgs);
  }

  void ProprioceptionNode::updateRobotModel(const nao_lola_sensor_msgs::msg::JointData& joint_data_msg) {
    measured_robot_model_.setJointPositions(joint_data_msg.positions);
    robot_model_publisher_->publish(nomadz_kinematics::packRobotModel(measured_robot_model_, this->now()));
  }
} // namespace nomadz_proprioception

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nomadz_proprioception::ProprioceptionNode)

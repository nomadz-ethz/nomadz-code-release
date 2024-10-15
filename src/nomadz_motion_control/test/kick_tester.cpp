#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp> // for visualizing pose in rviz

#include "nomadz_motion_control_msgs/msg/motion_request.hpp"
#include "nao_lola_command_msgs/msg/joint_requests.hpp"
#include "nao_lola_sensor_msgs/msg/joint_data.hpp"
#include "nomadz_motion_control/kick_engine/kick_engine.hpp"

using namespace std::chrono_literals;
using nomadz_core::constants::MOTION_CYCLE_TIME;
using nomadz_motion_control::KickEngine;
using MarkerMsgT = visualization_msgs::msg::Marker;
using JointRequestsMsgT = nao_lola_command_msgs::msg::JointRequests;
using JointDataMsgT = nao_lola_sensor_msgs::msg::JointData;
using MotionRequestMsgT = nomadz_motion_control_msgs::msg::MotionRequest;

class KickTester : public rclcpp::Node {
public:
  KickTester() : Node("kick_tester"), kicker_(std::make_unique<KickEngine>(rclcpp::NodeOptions())) {
    joint_requests_publisher_ = this->create_publisher<JointRequestsMsgT>("effectors/joint_requests", 10);
    joint_data_subscriber_ = this->create_subscription<JointDataMsgT>(
      "sensors/joint_data", 100, [this](const JointDataMsgT::SharedPtr msg) { this->topicCallback(msg); });
    motion_request_subscriber_ = this->create_subscription<MotionRequestMsgT>(
      "behavior/motion_request", 100, [this](const MotionRequestMsgT::SharedPtr msg) { motion_request_ = *msg; });
    marker_publisher_ = this->create_publisher<MarkerMsgT>("visualization/marker", 10);
    timer_ = this->create_wall_timer(std::chrono::duration<float>(MOTION_CYCLE_TIME), [this] { timerCallback(); });
    com_ = getMarker(1, MarkerMsgT::SPHERE);

    left_foot_ = getMarker(2, MarkerMsgT::ARROW);
    right_foot_ = getMarker(3, MarkerMsgT::ARROW);
  }

private:
  rclcpp::Subscription<JointDataMsgT>::SharedPtr joint_data_subscriber_;
  rclcpp::Subscription<MotionRequestMsgT>::SharedPtr motion_request_subscriber_;

  rclcpp::Publisher<JointRequestsMsgT>::SharedPtr joint_requests_publisher_;
  rclcpp::Publisher<MarkerMsgT>::SharedPtr marker_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
  MarkerMsgT com_, left_foot_, right_foot_;
  MotionRequestMsgT motion_request_;

  std::unique_ptr<KickEngine> kicker_;

  void timerCallback() {
    if (!kicker_->kickExecuted()) {
      kicker_->executeControlLoop(motion_request_);
      joint_requests_publisher_->publish(kicker_->current_request_msg);
    } else {
      joint_requests_publisher_->publish(JointRequestsMsgT());
      kicker_->publicReset();
    }
  }

  void topicCallback(const JointDataMsgT::SharedPtr msg) {
    kicker_->robot_model->setJointPositions(msg->positions);
    updateMarker(com_, kicker_->robot_model->getCenterOfMass());
    marker_publisher_->publish(com_);
    updateMarker(left_foot_, kicker_->robot_model->getFootPose(true));
    updateMarker(right_foot_, kicker_->robot_model->getFootPose(false));
    marker_publisher_->publish(left_foot_);
    marker_publisher_->publish(right_foot_);
  }

  MarkerMsgT getMarker(const int id, const int type) {
    MarkerMsgT marker;
    marker.header.frame_id = "torso";
    marker.header.stamp = this->get_clock().get()->now();
    marker.id = id;
    marker.type = type;
    marker.action = MarkerMsgT::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 5e-2;
    marker.scale.y = 5e-2;
    marker.scale.z = 5e-2;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    return marker;
  }

  static void updateMarker(MarkerMsgT& marker, Eigen::Vector3f position) {
    marker.action = MarkerMsgT::MODIFY;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
  }

  static void updateMarker(MarkerMsgT& marker, Eigen::Affine3f pose) {
    marker.action = MarkerMsgT::MODIFY;
    const auto position = pose.translation();
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    const Eigen::Quaternionf rotation{pose.rotation()};
    marker.pose.orientation.x = rotation.x();
    marker.pose.orientation.y = rotation.y();
    marker.pose.orientation.z = rotation.z();
    marker.pose.orientation.w = rotation.w();
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KickTester>());
  rclcpp::shutdown();
  return 0;
}

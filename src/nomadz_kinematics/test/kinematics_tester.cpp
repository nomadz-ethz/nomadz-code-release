#include <chrono>
#include <iostream>
#include <memory>
#include <random>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include "nao_lola_command_msgs/msg/joint_requests.hpp"
#include "nomadz_definitions/joint_indexes.hpp"
#include "nomadz_kinematics/inverse_kinematics.hpp"
#include "nomadz_kinematics/robot_dimensions.hpp"
#include "nomadz_kinematics/forward_kinematics.hpp"

using namespace std::chrono_literals;

using Eigen::Affine3f;
using Eigen::AngleAxisf;
using Eigen::Quaternionf;
using Eigen::Translation3f;
using Eigen::Vector3f;

namespace nk = nomadz_kinematics;
namespace joint_indexes = nomadz_definitions::joint_indexes;

class KinematicsTestNode : public rclcpp::Node {
public:
  KinematicsTestNode() : Node("webot_kinematics_test") {
    rclcpp::Clock const system_clock;

    timer_ = this->create_wall_timer(5s, [this] { timerCallback(); });
    joint_position_publisher_ =
      this->create_publisher<nao_lola_command_msgs::msg::JointRequests>("effectors/joint_requests", 10);
    poses_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("poses", 10);
  }
  ~KinematicsTestNode() override = default;

private:
  void testForwardKinematicsLeg() {
    std::array<float, 6> random_values;
    for (unsigned int i = 0; i < 6; ++i) {
      random_values[i] = static_cast<float>(rand()) * (1.F / static_cast<float>(RAND_MAX)) - 0.5F;
    }
    for (unsigned int i = joint_indexes::L_HIP_YAW_PITCH; i <= joint_indexes::L_ANKLE_ROLL; ++i) {
      current_joint_position_[i] = random_values[i - static_cast<int>(joint_indexes::L_HIP_YAW_PITCH)];
    }
    for (unsigned int i = joint_indexes::R_HIP_ROLL; i <= joint_indexes::R_ANKLE_ROLL; ++i) {
      current_joint_position_[i] = random_values[i - static_cast<int>(joint_indexes::R_HIP_ROLL) + 1];
    }
    auto tibia_l = nk::torsoFromPelvis(true, random_values[0]) * nk::pelvisFromHip(random_values[1]) *
                   nk::hipFromThigh(random_values[2]) * nk::thighFromTibia(random_values[3]);
    auto foot_l = nk::torsoFromPelvis(true, random_values[0]) * nk::pelvisFromHip(random_values[1]) *
                  nk::hipFromThigh(random_values[2]) * nk::thighFromTibia(random_values[3]) *
                  nk::tibiaFromAnkle(random_values[4]) * nk::ankleFromFoot(random_values[5]);

    auto tibia_r = nk::torsoFromPelvis(false, random_values[0]) * nk::pelvisFromHip(random_values[1]) *
                   nk::hipFromThigh(random_values[2]) * nk::thighFromTibia(random_values[3]);
    auto foot_r = nk::torsoFromPelvis(false, random_values[0]) * nk::pelvisFromHip(random_values[1]) *
                  nk::hipFromThigh(random_values[2]) * nk::thighFromTibia(random_values[3]) *
                  nk::tibiaFromAnkle(random_values[4]) * nk::ankleFromFoot(random_values[5]);

    auto pose = geometry_msgs::msg::Pose();
    for (auto link : {tibia_l, foot_l, tibia_r, foot_r}) {
      pose.position.x = link.translation().x();
      pose.position.y = link.translation().y();
      pose.position.z = link.translation().z();
      Quaternionf q(link.rotation());
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();
      poses_.poses.push_back(pose);
    }
  }

  void testInverseKinematicsLeg() {
    Affine3f left_foot_target;
    Affine3f right_foot_target;
    std::array<float, 6> random_values;
    for (unsigned int i = 0; i < 6; ++i) {
      random_values[i] = static_cast<float>(rand()) * (1.F / static_cast<float>(RAND_MAX));
    }
    float const yaw_l = 0.1F * (random_values[0] - 0.5F) * static_cast<float>(M_PI);
    float const x = 0.2F * random_values[1] - 0.1F;
    float const y = 0.1F * random_values[2] + 0.05F;
    float const z = -0.05F * random_values[3] - 0.19F;
    float const roll = 0.5F * (random_values[4]);
    float const pitch = -0.5F * (random_values[5]);

    left_foot_target = Translation3f(x, y, z) * AngleAxisf(yaw_l, Vector3f::UnitZ()) * AngleAxisf(pitch, Vector3f::UnitY()) *
                       AngleAxisf(roll, Vector3f::UnitX());
    right_foot_target = Translation3f(x, -y, z) * AngleAxisf(-yaw_l, Vector3f::UnitZ()) *
                        AngleAxisf(pitch, Vector3f::UnitY()) * AngleAxisf(-roll, Vector3f::UnitX());
    nk::inverseTorsoFromFoot(left_foot_target, right_foot_target, current_joint_position_);

    auto pose = geometry_msgs::msg::Pose();

    for (auto link : {left_foot_target, right_foot_target}) {
      pose.position.x = link.translation().x();
      pose.position.y = link.translation().y();
      pose.position.z = link.translation().z();
      Quaternionf q(link.rotation());
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();
      poses_.poses.push_back(pose);
    }
  }

  void timerCallback() {
    auto joint_position_req = nao_lola_command_msgs::msg::JointRequests();
    testInverseKinematicsLeg();
    for (unsigned int i = 0; i < joint_indexes::NUM_JOINTS; ++i) {
      joint_position_req.indexes.push_back(i);
      joint_position_req.positions.push_back(current_joint_position_[i]);
      joint_position_req.stiffnesses.push_back(1.F);
    }

    joint_position_publisher_->publish(joint_position_req);
    poses_.header.stamp = this->get_clock().get()->now();
    poses_.header.frame_id = "base_link";
    poses_publisher_->publish(poses_);
    poses_.poses.clear();
  }

  std::shared_ptr<rclcpp::TimerBase> timer_;
  std::shared_ptr<rclcpp::Publisher<nao_lola_command_msgs::msg::JointRequests>> joint_position_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseArray>> poses_publisher_;

  std::array<float, nk::NUM_DOF> current_joint_position_;
  geometry_msgs::msg::PoseArray poses_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicsTestNode>());
  rclcpp::shutdown();
  return 0;
}

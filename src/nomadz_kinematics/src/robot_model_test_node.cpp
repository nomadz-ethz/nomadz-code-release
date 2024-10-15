#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "nomadz_kinematics/robot_model.hpp"
#include "nao_lola_command_msgs/msg/joint_requests.hpp"
#include "nao_lola_sensor_msgs/msg/joint_data.hpp"
#include "nao_lola_sensor_msgs/msg/joint_indexes.hpp"

using namespace std::chrono_literals;
using nomadz_kinematics::RobotModel;
namespace joint_indexes = nomadz_definitions::joint_indexes;

class RobotModelTestNode : public rclcpp::Node {

public:
  RobotModelTestNode() : Node("robot_model_test_node") {
    generateRandomJointPositions(jp_);
    rm_.setJointPositions(jp_);
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("test_marker", 10);
    timer_ = this->create_wall_timer(2000ms, [this] { timerCallback(); });
    joint_position_publisher_ =
      this->create_publisher<nao_lola_command_msgs::msg::JointRequests>("effectors/joint_requests", 10);
    joint_position_subscription_ = this->create_subscription<nao_lola_sensor_msgs::msg::JointData>(
      "sensors/joint_data", 10, [this](nao_lola_sensor_msgs::msg::JointData::SharedPtr msg) { return topicCallback(msg); });

    marker_.header.frame_id = "torso";
    marker_.header.stamp = this->get_clock().get()->now();
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::SPHERE;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.pose.position.x = 0;
    marker_.pose.position.y = 0;
    marker_.pose.position.z = 0;
    marker_.pose.orientation.x = 0;
    marker_.pose.orientation.y = 0;
    marker_.pose.orientation.z = 0;
    marker_.pose.orientation.w = 0;
    marker_.scale.x = 1e-1;
    marker_.scale.y = 1e-1;
    marker_.scale.z = 1e-1;
    marker_.color.r = 1.0;
    marker_.color.g = 0.0;
    marker_.color.b = 0.0;
    marker_.color.a = 0.5;
  }

private:
  void topicCallback(const nao_lola_sensor_msgs::msg::JointData::SharedPtr msg) {
    std::array<float, joint_indexes::NUM_JOINTS> jp;
    for (unsigned int i = 0; i < joint_indexes::NUM_JOINTS; ++i) {
      jp[i] = msg->positions[i];
    }
    rm_.setJointPositions(jp);
    // Update pose
    Eigen::Vector3f center_of_mass = rm_.getCenterOfMass() * 1e3;
    marker_.action = visualization_msgs::msg::Marker::MODIFY;
    marker_.header.stamp = this->get_clock().get()->now();
    marker_.pose.position.x = center_of_mass.x();
    marker_.pose.position.y = center_of_mass.y();
    marker_.pose.position.z = center_of_mass.z();

    marker_publisher_->publish(marker_);
  }

  void timerCallback() {
    generateRandomJointPositions(jp_);

    auto jr_msg = nao_lola_command_msgs::msg::JointRequests();
    for (unsigned int i = 0; i < joint_indexes::NUM_JOINTS; ++i) {
      jr_msg.indexes.push_back(i);
      jr_msg.positions.push_back(jp_[i]);
      jr_msg.stiffnesses.push_back(1.0);
    }

    joint_position_publisher_->publish(jr_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  visualization_msgs::msg::Marker marker_;
  std::vector<visualization_msgs::msg::Marker> limbs_markers_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointRequests>::SharedPtr joint_position_publisher_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::JointData>::SharedPtr joint_position_subscription_;

  RobotModel rm_;
  std::array<float, joint_indexes::NUM_JOINTS> jp_;

  std::string getCenterOfMassInfo() {
    Eigen::Vector3f center_of_mass = rm_.getCenterOfMass();
    std::stringstream result;

    result << "Center of Mass: " << center_of_mass.transpose() << "\n";

    return result.str();
  }

  static void generateRandomJointPositions(std::array<float, joint_indexes::NUM_JOINTS>& joint_positions) {
    for (int i = 0; i < joint_indexes::NUM_JOINTS; i++) {
      joint_positions[i] =
        static_cast<float>(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 4.0 * M_PI - 2 * M_PI);
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotModelTestNode>());
  rclcpp::shutdown();
  return 0;
}

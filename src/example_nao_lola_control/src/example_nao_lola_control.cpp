#include "example_nao_lola_control.hpp"

#include <rclcpp/logger.hpp>

#include "nomadz_definitions/joint_indexes.hpp"

using namespace std::chrono_literals;
namespace joint_indexes = nomadz_definitions::joint_indexes;

namespace example_nao_lola_control {

  ExampleNaoLolaControlNode::ExampleNaoLolaControlNode() : Node("webot_example_control") {
    // Add standard publisher.
    joint_requests_publisher_ = this->create_publisher<JointRequestsMsgT>(JOINT_REQUESTS_TOPIC, 10);

    timer_ = this->create_wall_timer(5s, [this]() { timerCallback(); });

    // Add standard subscriber.
    robot_joint_data_sub_ = this->create_subscription<JointDataMsgT>(
      JOINT_DATA_TOPIC, 10, [this](std::shared_ptr<JointDataMsgT> msg) { robotJointPositionCallback(msg); });

    // Set log level to info.
    get_logger().set_level(rclcpp::Logger::Level::Info);
  }

  void ExampleNaoLolaControlNode::robotJointPositionCallback(std::shared_ptr<nao_lola_sensor_msgs::msg::JointData> msg) {
    for (unsigned int i = 0; i < joint_indexes::NUM_JOINTS; ++i) {
      current_joint_positions_[i] = msg->positions[i];
    }
  }

  void ExampleNaoLolaControlNode::timerCallback() {
    const std::array<uint8_t, 2> joints_to_control = {joint_indexes::R_SHOULDER_PITCH, joint_indexes::L_SHOULDER_PITCH};
    auto out_msg_positions = nao_lola_command_msgs::msg::JointRequests();

    for (uint8_t i : joints_to_control) {
      float current_elbow_setpoint = 0.F;
      if (last_setpoint_was_0_) {
        current_elbow_setpoint = elbow_setpoint_2_;
      } else {
        current_elbow_setpoint = elbow_setpoint_1_;
      }

      out_msg_positions.indexes.push_back(i);
      out_msg_positions.positions.push_back(current_elbow_setpoint);
      out_msg_positions.stiffnesses.push_back(1.F);
    }

    last_setpoint_was_0_ = !last_setpoint_was_0_;
    joint_requests_publisher_->publish(out_msg_positions);
  }
} // namespace example_nao_lola_control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<example_nao_lola_control::ExampleNaoLolaControlNode>());
  rclcpp::shutdown();
  return 0;
}

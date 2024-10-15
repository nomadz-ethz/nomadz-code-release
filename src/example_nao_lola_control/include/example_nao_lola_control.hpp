#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "nao_lola_sensor_msgs/msg/joint_data.hpp"
#include "nao_lola_command_msgs/msg/joint_requests.hpp"

namespace example_nao_lola_control {
  class ExampleNaoLolaControlNode : public rclcpp::Node {
    using JointDataMsgT = nao_lola_sensor_msgs::msg::JointData;
    using JointRequestsMsgT = nao_lola_command_msgs::msg::JointRequests;

    static constexpr const char* JOINT_DATA_TOPIC = "sensors/joint_data";

    static constexpr const char* JOINT_REQUESTS_TOPIC = "effectors/joint_requests";

  public:
    ExampleNaoLolaControlNode();
    ~ExampleNaoLolaControlNode() override = default;

  private:
    void robotJointPositionCallback(std::shared_ptr<JointDataMsgT> msg);
    std::shared_ptr<rclcpp::Subscription<JointDataMsgT>> robot_joint_data_sub_;

    void timerCallback();

    rclcpp::Clock system_clock_;
    std::shared_ptr<rclcpp::TimerBase> timer_;
    std::shared_ptr<rclcpp::Publisher<JointRequestsMsgT>> joint_requests_publisher_;

    std::array<float, 25> current_joint_positions_;

    bool last_setpoint_was_0_ = true;
    float elbow_setpoint_1_ = 0.F;
    float elbow_setpoint_2_ = 1.F;
  };
} // namespace example_nao_lola_control

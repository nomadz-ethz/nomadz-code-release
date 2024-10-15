#pragma once

#include <rclcpp/rclcpp.hpp>

#include "nao_lola_command_msgs/msg/chest_led.hpp"
#include "nao_lola_command_msgs/msg/ear_leds.hpp"
#include "nao_lola_command_msgs/msg/eye_leds.hpp"
#include "nomadz_behavior_msgs/msg/ego_status.hpp"
#include "nomadz_modeling_msgs/msg/world_model.hpp"
#include "nomadz_motion_control_msgs/msg/motion_info.hpp"
#include "nomadz_behavior_msgs/msg/game_status.hpp"
#include "nomadz_led_control/robot_game_states_enum.hpp"
#include "nomadz_motion_control_msgs/special_action_request_enums.hpp"
#include "nomadz_communication_msgs/robocup_game_control_data_enums.hpp"

namespace nomadz_led_control {

  class LEDHandler : public rclcpp::Node {
    using GameStatusMsgT = nomadz_behavior_msgs::msg::GameStatus;
    using EgoStatusMsgT = nomadz_behavior_msgs::msg::EgoStatus;
    using WorldModelMsgT = nomadz_modeling_msgs::msg::WorldModel;
    using MotionInfoMsgT = nomadz_motion_control_msgs::msg::MotionInfo;
    using ChestLedMsgT = nao_lola_command_msgs::msg::ChestLed;
    using EarLedsMsgT = nao_lola_command_msgs::msg::EarLeds;
    using EyeLedsMsgT = nao_lola_command_msgs::msg::EyeLeds;

  public:
    explicit LEDHandler(const rclcpp::NodeOptions& options);

  private:
    std::shared_ptr<rclcpp::TimerBase> blink_timer_;
    std::shared_ptr<rclcpp::TimerBase> gold_eye_timer_;
    bool is_blinking_ = false;
    bool is_blue_ = false;
    bool is_penalized_already_ = false;
    bool right_eye_has_ball_lock_ = false;
    bool left_eye_ball_model_valid_ = false;

    void createPublishers();
    void publishOnce();
    void createSubscribers();
    void startTimer();
    void stopTimer();
    void blueBlinking();
    void setChestButtonGameStateCallback(RobotGameStates robot_game_state);
    void setChestButtonPenalizedCallback();
    void setEyesCallback();
    void setEyesGold();

    // Subscribers
    rclcpp::Subscription<GameStatusMsgT>::SharedPtr game_status_sub_;
    rclcpp::Subscription<EgoStatusMsgT>::SharedPtr ego_status_sub_;
    rclcpp::Subscription<MotionInfoMsgT>::SharedPtr motion_info_sub_;
    rclcpp::Subscription<WorldModelMsgT>::SharedPtr world_model_sub_;

    // Publishers
    rclcpp::Publisher<ChestLedMsgT>::SharedPtr chest_led_pub_;
    rclcpp::Publisher<EarLedsMsgT>::SharedPtr ear_led_pub_;
    rclcpp::Publisher<EyeLedsMsgT>::SharedPtr eye_leds_pub_;
  };

} // namespace nomadz_led_control

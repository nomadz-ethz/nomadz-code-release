#include "nomadz_led_control/led_handler.hpp"

#include <std_msgs/msg/header.hpp>

#define INTERVAL_MS 200
#define NUM_EYE_LEDS 8

namespace nomadz_led_control {

  LEDHandler::LEDHandler(const rclcpp::NodeOptions& options) : Node("led_handler", options) {

    createPublishers();
    // publish once on every topic, to handle unintialized topics
    publishOnce();
    createSubscribers();

    // TODO(nicole) @Nicole change logic for blue blinking
    // blink_timer_ = create_wall_timer(std::chrono::milliseconds(INTERVAL_MS), [this] { blueBlinking(); });

    setEyesGold();
    gold_eye_timer_ = create_wall_timer(std::chrono::milliseconds(1000), [this] { setEyesGold(); });

    // blink_timer_->cancel();
  }

  void LEDHandler::createPublishers() {
    RCLCPP_DEBUG(get_logger(), "Initialise publishers");
    chest_led_pub_ = create_publisher<ChestLedMsgT>("effectors/chest_led", 1);
    ear_led_pub_ = create_publisher<EarLedsMsgT>("effectors/ear_led", 1);
    eye_leds_pub_ = create_publisher<EyeLedsMsgT>("effectors/eye_leds", 1);
    RCLCPP_DEBUG(get_logger(), "Finished initialising publishers");
  }

  void LEDHandler::publishOnce() {
    auto led_message = ChestLedMsgT();
    std_msgs::msg::ColorRGBA color;
    color.r = 0.F;
    color.g = 0.F;
    color.b = 0.F;
    led_message.color = color;
    led_message.header.stamp = this->now();
    chest_led_pub_->publish(led_message);

    auto eye_led_message = EyeLedsMsgT();
    for (int i = 0; i < NUM_EYE_LEDS; i++) {
      eye_led_message.right_colors[i] = color;
      eye_led_message.left_colors[i] = color;
    }
    eye_led_message.header.stamp = this->now();
    eye_leds_pub_->publish(eye_led_message);
  }

  void LEDHandler::createSubscribers() {
    RCLCPP_DEBUG(get_logger(), "Initialise subscribers");

    // map game states on LED colors (see SPL rulebook)
    game_status_sub_ =
      create_subscription<GameStatusMsgT>("behavior/game_status", 1, [this](const GameStatusMsgT::SharedPtr msg) {
        setChestButtonGameStateCallback(static_cast<RobotGameStates>(msg->game_state));
      });

    world_model_sub_ =
      create_subscription<WorldModelMsgT>("modeling/world_model", 1, [this](const WorldModelMsgT::SharedPtr msg) {
        if (msg->ball_model.valid) {
          left_eye_ball_model_valid_ = true;
          setEyesCallback();
        } else {
          left_eye_ball_model_valid_ = false;
        }
      });

    // map penalize on red chest LED color and ball lock on right eye red LED color
    ego_status_sub_ =
      create_subscription<EgoStatusMsgT>("behavior/ego_status", 1, [this](const EgoStatusMsgT::SharedPtr msg) {
        if (msg->is_penalized) {
          setChestButtonPenalizedCallback();
        } else {
          is_penalized_already_ = false;
        }

        if (msg->has_ball_lock) {
          right_eye_has_ball_lock_ = true;
          setEyesCallback();
        } else {
          right_eye_has_ball_lock_ = false;
        }
      });

    // // map unstiff on blue blinking
    // namespace sa = nomadz_motion_control_msgs::special_actions;
    // motion_info_sub_ = create_subscription<MotionInfoMsgT>(
    //   "motion_control/motion_info", 1, [this](MotionInfoMsgT::SharedPtr msg) {
    //     if (msg->executed_motion_request.special_action_request.special_action_type == sa::PLAY_DEAD) {
    //       startTimer();
    //     } else {
    //       stopTimer();
    //     }
    //   });

    RCLCPP_DEBUG(get_logger(), "Finished initialising subscribers");
  }

  void LEDHandler::startTimer() {
    if (!is_blinking_) {
      is_blinking_ = true;
      blink_timer_->reset();
    }
  }

  void LEDHandler::stopTimer() {
    if (is_blinking_) {
      is_blinking_ = false;
      blink_timer_->cancel();
      setChestButtonGameStateCallback(RobotGameStates::INITIAL);
    }
  }

  void LEDHandler::blueBlinking() {
    if (is_blinking_) {
      if (is_blue_) {
        setChestButtonGameStateCallback(RobotGameStates::INITIAL); // turn off
      } else {
        setChestButtonGameStateCallback(RobotGameStates::UNSTIFF); // turn on blue
      }
      is_blue_ = !is_blue_; // toggle state
    }
  }

  void LEDHandler::setEyesGold() {
    // set eyes gold is not prioritzed
    if (!right_eye_has_ball_lock_ && !left_eye_ball_model_valid_) {
      auto led_message = EyeLedsMsgT();
      std_msgs::msg::ColorRGBA color;
      color.r = 0.98F;
      color.g = 0.98F;
      color.b = 0.8;

      for (int i = 0; i < NUM_EYE_LEDS; i++) {
        led_message.right_colors[i] = color;
        led_message.left_colors[i] = color;
      }
      led_message.header.stamp = this->now();
      eye_leds_pub_->publish(led_message);
    } else {
      return;
    }
  }

  void LEDHandler::setEyesCallback() {
    auto led_message = EyeLedsMsgT();
    led_message.header.stamp = this->now();

    if (right_eye_has_ball_lock_) {
      std_msgs::msg::ColorRGBA color;
      color.r = 1.F;
      color.g = 0.F;
      color.b = 0.F;

      for (int i = 0; i < NUM_EYE_LEDS; i++) {
        led_message.right_colors[i] = color;
      }
    }

    if (left_eye_ball_model_valid_) {
      std_msgs::msg::ColorRGBA color;
      color.r = 0.F;
      color.g = 0.F;
      color.b = 1.F;

      for (int i = 0; i < NUM_EYE_LEDS; i++) {
        led_message.left_colors[i] = color;
      }
    }
    eye_leds_pub_->publish(led_message);
  }

  void LEDHandler::setChestButtonPenalizedCallback() {
    auto led_message = ChestLedMsgT();
    std_msgs::msg::ColorRGBA color;
    color.r = 1.F;
    color.g = 0.F;
    color.b = 0.F;
    led_message.color = color;
    led_message.header.stamp = this->now();
    chest_led_pub_->publish(led_message);
    is_penalized_already_ = true;
  }

  /* SPL Rules for chest button LED colors:
   • Unstiff: Blue-Blinking
   • Initial: Off
   • Standby: Cyan
   • Ready: Blue
   • Set: Yellow
   • Playing: Green
   • Penalized: Red
   • Finished: Off
   • Calibration: Purple
   */
  void LEDHandler::setChestButtonGameStateCallback(RobotGameStates robot_game_state) {
    auto led_message = ChestLedMsgT();
    std_msgs::msg::ColorRGBA color;

    // gamestate should not overwrite penalized state color
    if (is_penalized_already_) {
      setChestButtonPenalizedCallback();
      return;
    }

    is_penalized_already_ = false;

    switch (robot_game_state) {

    case RobotGameStates::INITIAL:
      color.r = 0.F;
      color.g = 0.F;
      color.b = 0.F;
      break;

    case RobotGameStates::READY:
      color.r = 0.F;
      color.g = 0.F;
      color.b = 1.F;
      break;

    case RobotGameStates::SET:
      color.r = 1.F;
      color.g = 1.F;
      color.b = 0.F;
      break;

    case RobotGameStates::PLAYING:
      color.r = 0.F;
      color.g = 1.F;
      color.b = 0.F;
      break;

    case RobotGameStates::FINISHED:
      color.r = 0.F;
      color.g = 0.F;
      color.b = 0.F;
      break;

    case RobotGameStates::STANDBY:
      color.r = 0.F;
      color.g = 1.F;
      color.b = 1.F;
      break;

    // UNSTIFF switches between case UNSTIFF (blue) and case INITIAL (off) to trigger blue blinking
    case RobotGameStates::UNSTIFF:
      color.r = 0.F;
      color.g = 0.F;
      color.b = 1.F;
      break;

    default:
      color.r = 0.F;
      color.g = 0.F;
      color.b = 0.F;
      break;
    }

    led_message.color = color;
    led_message.header.stamp = this->now();
    chest_led_pub_->publish(led_message);
  }

} // namespace nomadz_led_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nomadz_led_control::LEDHandler)

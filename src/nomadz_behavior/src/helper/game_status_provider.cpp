#include "nomadz_behavior/helper/game_status_provider.hpp"

#include "nomadz_communication_msgs/robocup_game_control_data_enums.hpp"
#include "nomadz_behavior/data_types/game_status.hpp"

namespace nomadz_behavior {

  GameStatusProvider::GameStatusProvider(const rclcpp::NodeOptions& options) : Node("game_status_provider", options) {

    detection_sub_ =
      create_subscription<WhistleDetectionMsgT>(WISTLE_DETECTION_TOPIC, 1, [this](WhistleDetectionMsgT::ConstSharedPtr msg) {
        if (msg->local_whistle_detected || msg->team_whistle_detected) {
          whistle_detected_ = true;
        }
        if (msg->local_referee_gesture_detected || msg->team_referee_gesture_detected) {
          gesture_detected_ = true;
        }
      });

    gc_data_sub_ = create_subscription<GCDataMsgT>(GC_DATA_TOPIC, 1, [this](GCDataMsgT::ConstSharedPtr msg) {
      GameStatusMsgT game_status_msg;
      game_status_msg.header.stamp = this->now();
      game_status_msg.game_phase = msg->game_phase;
      game_status_msg.game_state = predictGameStatus(msg->state);

      game_status_msg.set_play = msg->set_play;
      game_status_msg.is_first_half = msg->first_half;
      game_status_msg.is_kicking_team = msg->kicking_team;
      game_status_pub_->publish(game_status_msg);
    });

    game_status_pub_ = create_publisher<GameStatusMsgT>(GAME_STATUS_TOPIC, 1);
  }

  uint8_t GameStatusProvider::predictGameStatus(uint8_t raw_game_state) {
    switch (static_cast<GameStatus::GameState>(raw_game_state)) {
    case GameStatus::GameState::INITIAL:
    case GameStatus::GameState::FINISHED:
    case GameStatus::GameState::READY:
      gesture_detected_ = false;
      break;
    case GameStatus::GameState::STANDBY:
      if (gesture_detected_) {
        return static_cast<uint8_t>(GameStatus::GameState::READY);
      }
      break;
    case GameStatus::GameState::SET:
      if (whistle_detected_) {
        return static_cast<uint8_t>(GameStatus::GameState::PLAYING);
      }
      break;
    case GameStatus::GameState::PLAYING:
      whistle_detected_ = false;
      break;
    default:
      break;
    }
    return raw_game_state;
  }

} // namespace nomadz_behavior

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nomadz_behavior::GameStatusProvider)

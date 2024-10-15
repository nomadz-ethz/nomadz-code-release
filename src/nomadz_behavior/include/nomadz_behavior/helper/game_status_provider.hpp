#pragma once

#include <rclcpp/rclcpp.hpp>

#include "nomadz_behavior_msgs/msg/game_status.hpp"
#include "nomadz_audio_processing_msgs/msg/whistle_detection.hpp"
#include "nomadz_communication_msgs/msg/robocup_game_control_data.hpp"

namespace nomadz_behavior {

  class GameStatusProvider : public rclcpp::Node {
    using GCDataMsgT = nomadz_communication_msgs::msg::RobocupGameControlData;
    using WhistleDetectionMsgT = nomadz_audio_processing_msgs::msg::WhistleDetection;

    using GameStatusMsgT = nomadz_behavior_msgs::msg::GameStatus;

    static constexpr const char* GC_DATA_TOPIC = "communication/gc_data";
    static constexpr const char* WISTLE_DETECTION_TOPIC = "audio_processing/whistle_detection";

    static constexpr const char* GAME_STATUS_TOPIC = "behavior/game_status";

  public:
    explicit GameStatusProvider(const rclcpp::NodeOptions& options);

    uint8_t predictGameStatus(uint8_t raw_game_state);

  private:
    bool whistle_detected_{false};
    bool gesture_detected_{false};

    rclcpp::Subscription<GCDataMsgT>::SharedPtr gc_data_sub_;
    rclcpp::Subscription<WhistleDetectionMsgT>::SharedPtr detection_sub_;

    rclcpp::Publisher<GameStatusMsgT>::SharedPtr game_status_pub_;
  };
} // namespace nomadz_behavior

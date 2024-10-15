#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp> // Replace with appropriate message type

#include "nomadz_audio_processing/audio_provider.hpp"
#include "nomadz_audio_processing/whistle_recognizer.hpp"
#include "nomadz_audio_processing_msgs/msg/whistle_detection.hpp"
#include "nomadz_communication_msgs/msg/team_comm_info.hpp"
#include "nomadz_configuration/game_settings.hpp"

namespace nomadz_audio_processing {

  class AudioProcessor : public rclcpp::Node {
    using DetectionMsgT = nomadz_audio_processing_msgs::msg::WhistleDetection;
    using TeamCommInfoMsgT = nomadz_communication_msgs::msg::TeamCommInfo;

    static constexpr int MAX_NUM_OF_PLAYERS = 7;

    static constexpr const char* DEFAULT_NODE_NAME = "audio_processor";

  public:
    explicit AudioProcessor(const std::string& node_name, const rclcpp::NodeOptions& options);
    explicit AudioProcessor(const rclcpp::NodeOptions& options);

  private:
    void setupPublishersAndSubscribers();
    void audioSetup();
    void captureAndPublishAudio();

    nomadz_configuration::GameSettings game_settings_;
    DetectionMsgT detection_msg_;

    std::shared_ptr<AudioDataProvider> audio_provider_;
    std::shared_ptr<WhistleRecognizer> whistle_recognizer_;

    std::shared_ptr<rclcpp::TimerBase> timer_;
    rclcpp::Subscription<TeamCommInfoMsgT>::SharedPtr team_comm_info_sub_;

    rclcpp::Publisher<DetectionMsgT>::SharedPtr detection_pub_;
  };

} // namespace nomadz_audio_processing

#include "nomadz_audio_processing/audio_processor.hpp"

#include <cstdint>
#include <chrono>
#include <filesystem>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/int32.hpp>

#include "nomadz_audio_processing/audio_provider.hpp"
#include "nomadz_configuration/io.hpp"

using namespace std::chrono_literals;

namespace nomadz_audio_processing {

  AudioProcessor::AudioProcessor(const rclcpp::NodeOptions& options) : AudioProcessor(DEFAULT_NODE_NAME, options) {}

  AudioProcessor::AudioProcessor(const std::string& node_name, const rclcpp::NodeOptions& options)
      : rclcpp::Node(node_name, options) {
    RCLCPP_INFO(get_logger(), "AudioProcessor: Starting initialization");
    // Setup publishers and subscribers
    game_settings_ = nomadz_configuration::getGameSettings();
    audioSetup();
    setupPublishersAndSubscribers();
    timer_ = create_wall_timer(250ms, [this]() { captureAndPublishAudio(); });
    captureAndPublishAudio();
  }

  void AudioProcessor::setupPublishersAndSubscribers() {
    team_comm_info_sub_ = create_subscription<TeamCommInfoMsgT>(
      "communication/team_comm_info", 1, [this](const TeamCommInfoMsgT::SharedPtr msg) {
        int num_of_detected_whistle_by_teammates = 0;
        for (int i = 0; i < MAX_NUM_OF_PLAYERS; i++) {
          if (i == game_settings_.player_id - 1) {
            continue;
          }
          if (msg->team_comm_info[i].ego_status.whistle_detected) {
            num_of_detected_whistle_by_teammates++;
          }
        }
        if (num_of_detected_whistle_by_teammates >= 2) {
          detection_msg_.team_whistle_detected = true;
        } else {
          detection_msg_.team_whistle_detected = false;
        }
      });
    detection_pub_ = create_publisher<DetectionMsgT>("audio_processing/whistle_detection", 1);
  }

  void AudioProcessor::audioSetup() {
    audio_provider_ = std::make_shared<AudioDataProvider>();

    std::filesystem::path whistle_model_path =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("nomadz_audio_processing")) /
      "models/whistle/whistle-recognizer-fast-20230630.tflite";
    whistle_recognizer_ = std::make_shared<WhistleRecognizer>(whistle_model_path);
  }

  void AudioProcessor::captureAndPublishAudio() {
    AudioDataProvider::AudioSample audio_sample = audio_provider_->updateData();

    std::vector<short> audio_data(audio_sample.samples.size());
    std::transform(audio_sample.samples.begin(), audio_sample.samples.end(), audio_data.begin(), [](float value) {
      return static_cast<short>(value);
    });
    detection_msg_.header.stamp = this->now();
    detection_msg_.local_whistle_detected = whistle_recognizer_->update(audio_data);

    detection_pub_->publish(detection_msg_);
  }

} // namespace nomadz_audio_processing

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nomadz_audio_processing::AudioProcessor)

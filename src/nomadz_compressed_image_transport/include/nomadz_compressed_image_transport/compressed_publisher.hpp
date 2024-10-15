#pragma once

#include <string>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/simple_publisher_plugin.hpp>

#include <rclcpp/node.hpp>

namespace nomadz_compressed_image_transport {
  using CompressedImage = sensor_msgs::msg::CompressedImage;

  class CompressedPublisher final : public image_transport::SimplePublisherPlugin<CompressedImage> {
  public:
    CompressedPublisher() : logger_(rclcpp::get_logger("NomadzCompressedPublisher")) {}

    std::string getTransportName() const override { return "nomadz_compressed"; }

  protected:
    // Overridden to set up reconfigure server
    void advertiseImpl(rclcpp::Node* node, const std::string& base_topic, rmw_qos_profile_t custom_qos) override;

    void publish(const sensor_msgs::msg::Image& message, const PublishFn& publish_fn) const override;

  private:
    int jpeg_quality_;
    rclcpp::Logger logger_;
  };

} // namespace nomadz_compressed_image_transport

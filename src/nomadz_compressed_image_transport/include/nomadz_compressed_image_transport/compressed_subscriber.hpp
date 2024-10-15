#pragma once

#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/subscription_options.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/simple_subscriber_plugin.hpp>

namespace nomadz_compressed_image_transport {

  class CompressedSubscriber final : public image_transport::SimpleSubscriberPlugin<sensor_msgs::msg::CompressedImage> {
  public:
    CompressedSubscriber() : logger_(rclcpp::get_logger("NomadzCompressedSubscriber")) {}

    std::string getTransportName() const override { return "nomadz_compressed"; }

  protected:
    // Overridden to set up reconfigure server
    void subscribeImpl(rclcpp::Node* /*node*/,
                       const std::string& base_topic,
                       const Callback& callback,
                       rmw_qos_profile_t custom_qos) override;

    void subscribeImpl(rclcpp::Node* /*node*/,
                       const std::string& base_topic,
                       const Callback& callback,
                       rmw_qos_profile_t custom_qos,
                       rclcpp::SubscriptionOptions options) override;

    void internalCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& message,
                          const Callback& user_cb) override;

  private:
    rclcpp::Logger logger_;
    rclcpp::Node* node_;
  };

} // namespace nomadz_compressed_image_transport

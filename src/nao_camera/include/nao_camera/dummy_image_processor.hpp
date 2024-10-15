#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace nao_camera {

  class DummyImageProcessor : public rclcpp::Node {
    using ImageMsgT = sensor_msgs::msg::Image;

    static constexpr const char* IMAGE_TOPIC = "image";

    static constexpr const char* PROCESSED_IMAGE_TOPIC = "image_processed";

  public:
    explicit DummyImageProcessor(const rclcpp::NodeOptions& options);

    DummyImageProcessor(const std::string& node_name, const rclcpp::NodeOptions& options);

  private:
    rclcpp::Subscription<ImageMsgT>::SharedPtr image_sub_;
    rclcpp::Publisher<ImageMsgT>::SharedPtr image_pub_;
  };

} // namespace nao_camera

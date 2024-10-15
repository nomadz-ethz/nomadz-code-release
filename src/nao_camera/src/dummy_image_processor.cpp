#include "nao_camera/dummy_image_processor.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace nao_camera {

  DummyImageProcessor::DummyImageProcessor(const rclcpp::NodeOptions& options)
      : DummyImageProcessor("dummy_image_processor", options) {}

  DummyImageProcessor::DummyImageProcessor(const std::string& node_name, const rclcpp::NodeOptions& options)
      : Node(node_name, options) {
    image_pub_ = create_publisher<ImageMsgT>("image_processed", 1);

    image_sub_ = create_subscription<ImageMsgT>("image", 1, [this](ImageMsgT::UniquePtr msg) {
      RCLCPP_INFO(get_logger(), "Received image with timestamp: %d", msg->header.stamp.sec);
      // NOLINTNEXTLINE(clang-analyzer-cplusplus.Move)
      image_pub_->publish(std::move(msg));
    });
  }

} // namespace nao_camera

RCLCPP_COMPONENTS_REGISTER_NODE(nao_camera::DummyImageProcessor)

#include "nomadz_compressed_image_transport/compressed_subscriber.hpp"

#include <limits>
#include <vector>
#include <cstring>

#include <rclcpp/parameter_client.hpp>

extern "C" {
#include <jpeglib.h>
}

using CompressedImage = sensor_msgs::msg::CompressedImage;

namespace nomadz_compressed_image_transport {

  void CompressedSubscriber::subscribeImpl(rclcpp::Node* node,
                                           const std::string& base_topic,
                                           const Callback& callback,
                                           rmw_qos_profile_t custom_qos) {
    this->subscribeImpl(node, base_topic, callback, custom_qos, rclcpp::SubscriptionOptions{});
  }

  void CompressedSubscriber::subscribeImpl(rclcpp::Node* node,
                                           const std::string& base_topic,
                                           const Callback& callback,
                                           rmw_qos_profile_t custom_qos,
                                           rclcpp::SubscriptionOptions options) {
    node_ = node;
    logger_ = node->get_logger();
    using Base = image_transport::SimpleSubscriberPlugin<CompressedImage>;
    Base::subscribeImplWithOptions(node, base_topic, callback, custom_qos, options);
  }

  void CompressedSubscriber::internalCallback(const CompressedImage::ConstSharedPtr& message, const Callback& user_cb) {
    // Initialize the JPEG decompression object
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);

    jpeg_mem_src(&cinfo, const_cast<unsigned char*>(message->data.data()), message->data.size());
    jpeg_read_header(&cinfo, TRUE);
    jpeg_start_decompress(&cinfo);

    // Allocate memory for the decompressed image
    unsigned width = cinfo.output_width;
    unsigned height = cinfo.output_height;
    unsigned pixel_size = cinfo.output_components;
    unsigned row_stride = width * pixel_size;

    std::size_t bmp_size = static_cast<std::size_t>(height) * row_stride;
    auto* bmp_buffer = static_cast<unsigned char*>(std::malloc(bmp_size));

    while (cinfo.output_scanline < cinfo.output_height) {
      unsigned char* buffer_array[1];
      buffer_array[0] = bmp_buffer + static_cast<size_t>(cinfo.output_scanline * row_stride);

      jpeg_read_scanlines(&cinfo, buffer_array, 1);
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);

    // Create a shared pointer to the image message
    auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
    image_msg->header = message->header;
    image_msg->height = height;
    image_msg->width = width;
    image_msg->encoding = "rgb8";
    image_msg->is_bigendian = 0U;
    image_msg->step = width * pixel_size;
    image_msg->data.resize(bmp_size);
    std::memcpy(image_msg->data.data(), bmp_buffer, bmp_size);

    // Pass the decompressed image data to the user callback
    user_cb(image_msg);

    free(bmp_buffer);
  }

} // namespace nomadz_compressed_image_transport

#include "nomadz_compressed_image_transport/compressed_publisher.hpp"

#include <cstring>

#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/parameter_client.hpp>

#include <sensor_msgs/image_encodings.hpp>

extern "C" {
#include <jpeglib.h>
}

namespace nomadz_compressed_image_transport {

  constexpr int DEFAULT_JPEG_QUALITY = 95;

  void CompressedPublisher::advertiseImpl(rclcpp::Node* node, const std::string& base_topic, rmw_qos_profile_t custom_qos) {
    using Base = image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage>;
    Base::advertiseImpl(node, base_topic, custom_qos);

    uint ns_len = node->get_effective_namespace().length();
    std::string param_base_name = base_topic.substr(ns_len);
    std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

    std::string jpeg_quality_param_name = param_base_name + ".jpeg_quality";
    rcl_interfaces::msg::ParameterDescriptor jpeg_quality_description;
    jpeg_quality_description.name = "jpeg_quality";
    jpeg_quality_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    jpeg_quality_description.description = "Image quality for JPEG format";
    jpeg_quality_description.read_only = false;
    rcl_interfaces::msg::IntegerRange jpeg_range;
    jpeg_range.from_value = 1;
    jpeg_range.to_value = 100;
    jpeg_range.step = 1;
    jpeg_quality_description.integer_range.push_back(jpeg_range);
    try {
      jpeg_quality_ =
        static_cast<int>(node->declare_parameter(jpeg_quality_param_name, DEFAULT_JPEG_QUALITY, jpeg_quality_description));
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
      RCLCPP_DEBUG(logger_, "%s was previously declared", jpeg_quality_param_name.c_str());
      jpeg_quality_ = static_cast<int>(node->get_parameter(jpeg_quality_param_name).get_value<int64_t>());
    }
  }
  // found something online https://gist.github.com/royshil/fa98604b01787172b270
  void CompressedPublisher::publish(const sensor_msgs::msg::Image& message, const PublishFn& publish_fn) const {
    // Check that the image encoding in yuv422_yuy2
    if (message.encoding != sensor_msgs::image_encodings::YUV422_YUY2) {
      RCLCPP_ERROR(logger_, "nomadz_compressed only supports yuv422_yuy2 images");
      return;
    }

    // Pack compressed image message
    sensor_msgs::msg::CompressedImage compressed;
    compressed.header = message.header;
    compressed.format = message.encoding + "; jpeg compressed ";

    // Initialize the JPEG compression object
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    // Set the output buffer
    unsigned char* outbuffer = nullptr;
    unsigned long outsize = 0;
    jpeg_mem_dest(&cinfo, &outbuffer, &outsize);

    // Set the compression parameters
    cinfo.image_width = message.width;
    cinfo.image_height = message.height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_YCbCr;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, DEFAULT_JPEG_QUALITY, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    // Write the image data to the output buffer
    std::vector<uint8_t> tmprowbuf(static_cast<std::size_t>(cinfo.image_width * 3U));
    JSAMPROW row_pointer[1];
    row_pointer[0] = tmprowbuf.data();
    while (cinfo.next_scanline < cinfo.image_height) {
      std::size_t offset =
        static_cast<std::size_t>(cinfo.next_scanline) * cinfo.image_width * 2U; // offset to the correct row
      for (std::size_t i = 0, j = 0; i < static_cast<std::size_t>(cinfo.image_width) * 2U;
           i += 4, j += 6) {                             // input strides by 4 bytes, output strides by 6 (2 pixels)
        tmprowbuf[j + 0] = message.data[offset + i + 0]; // Y (unique to this pixel)
        tmprowbuf[j + 1] = message.data[offset + i + 1]; // U (shared between pixels)
        tmprowbuf[j + 2] = message.data[offset + i + 3]; // V (shared between pixels)
        tmprowbuf[j + 3] = message.data[offset + i + 2]; // Y (unique to this pixel)
        tmprowbuf[j + 4] = message.data[offset + i + 1]; // U (shared between pixels)
        tmprowbuf[j + 5] = message.data[offset + i + 3]; // V (shared between pixels)
      }
      jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    // Finish the compression process
    jpeg_finish_compress(&cinfo);

    // Clean up the libjpeg objects
    jpeg_destroy_compress(&cinfo);

    // Set the compressed image data
    compressed.data.resize(outsize);
    std::memcpy((compressed.data).data(), outbuffer, outsize);
    compressed.header.stamp = rclcpp::Clock().now();

    // Publish the compressed image message
    publish_fn(compressed);

    // Clean up the output buffer
    free(outbuffer);
  }

} // namespace nomadz_compressed_image_transport

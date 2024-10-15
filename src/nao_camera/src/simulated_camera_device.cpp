#include <chrono>
#include <iostream>
#include <string>

#include <rclcpp/utilities.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "nao_camera/simulated_camera_device.hpp"

namespace nao_camera {

  SimulatedCameraDevice::SimulatedCameraDevice(const std::string& addr, int port) : socket_(io_service_) {
    for (unsigned char i = 0; i < 5; ++i) {
      try {
        socket_.connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(addr), port));
        std::cout << "Camera connection on " << addr << ":" << port << " successful.\n";
        return;
      } catch (boost::system::system_error& e) {
        std::cerr << "Could not establish TCP connection to " << addr << ":" << port << ". Retrying in 5 seconds.\n.";
        rclcpp::sleep_for(std::chrono::seconds(5));
      }
    }
    std::cerr << "Could not establish TCP connection to " << addr << ":" << port << "\n.";
    throw;
  }

  sensor_msgs::msg::Image::UniquePtr SimulatedCameraDevice::capture([[maybe_unused]] int timeout) {
    sensor_msgs::msg::Image::UniquePtr image_msg = std::make_unique<sensor_msgs::msg::Image>();
    if (!capture(*image_msg, timeout)) {
      return nullptr;
    }
    return image_msg;
  }

  bool SimulatedCameraDevice::capture(sensor_msgs::msg::Image& image_msg, [[maybe_unused]] int timeout) {
    boost::system::error_code error;

    auto is_asio_error = [&error]() -> bool {
      if (error && error != boost::asio::error::eof) {
        std::cout << "receive failed: " << error.message() << ".\n";
        return true;
      }
      return false;
    };

    boost::asio::streambuf receive_buffer;
    Header header;

    while (true) {
      boost::asio::read(socket_, receive_buffer, boost::asio::transfer_exactly(sizeof(Header)), error);
      if (is_asio_error()) {
        return false;
      }

      receiveHeader(receive_buffer, header);

      if (std::strncmp(header.magic_value, MAGIC_VALUE, 7) == 0) {
        break;
      }
    }

    const uint16_t step = header.width * 2;
    const uint32_t image_data_size = header.height * step;
    boost::asio::read(socket_, receive_buffer, boost::asio::transfer_exactly(image_data_size), error);
    if (is_asio_error()) {
      return false;
    }

    image_msg.data.resize(image_data_size);
    receiveImage(receive_buffer, image_msg.data);

    auto timestamp = std::chrono::system_clock::now();
    image_msg.header.stamp.sec =
      static_cast<int32_t>(std::chrono::duration_cast<std::chrono::seconds>(timestamp.time_since_epoch()).count());
    image_msg.header.stamp.nanosec = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch()).count() % 1000000000UL);

    image_msg.height = header.height;
    image_msg.width = header.width;
    image_msg.encoding = sensor_msgs::image_encodings::YUV422_YUY2;
    image_msg.step = image_msg.width * 2;

    return true;
  }

} // namespace nao_camera

#include "nao_camera/fake_camera_device.hpp"

#include <cstddef>
#include <thread>

#include <sensor_msgs/image_encodings.hpp>

namespace nao_camera {

  FakeCameraDevice::FakeCameraDevice(unsigned int width, unsigned int height, unsigned int fps)
      : width_(static_cast<uint16_t>(width)), height_(static_cast<uint16_t>(height)), fps_(static_cast<uint16_t>(fps)) {}

  sensor_msgs::msg::Image::UniquePtr FakeCameraDevice::capture(int timeout) {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    if (!capture(*msg, timeout)) {
      return nullptr;
    }

    return msg;
  }

  bool FakeCameraDevice::capture(sensor_msgs::msg::Image& image_msg, [[maybe_unused]] int timeout) {
    image_msg.height = height_;
    image_msg.width = width_;
    image_msg.encoding = sensor_msgs::image_encodings::YUV422_YUY2;
    image_msg.step = step_;
    image_msg.data.resize(image_data_size_);

    // calculate the expected time of the next frame
    std::chrono::steady_clock::time_point expected_next_capture_time;
    if (last_capture_time_.time_since_epoch().count() == 0) {
      last_capture_time_ = std::chrono::steady_clock::now();
      expected_next_capture_time = last_capture_time_;
    } else {
      expected_next_capture_time = last_capture_time_ + std::chrono::milliseconds(1000 / fps_);
      auto current_time = std::chrono::steady_clock::now();
      if (current_time > expected_next_capture_time) {
        expected_next_capture_time = current_time;
      } else {
        std::this_thread::sleep_until(expected_next_capture_time);
      }
    }

    image_msg.header.stamp.sec = static_cast<int32_t>(
      std::chrono::duration_cast<std::chrono::seconds>(expected_next_capture_time.time_since_epoch()).count());
    image_msg.header.stamp.nanosec = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(expected_next_capture_time.time_since_epoch()).count() %
      1000000000UL);

    last_capture_time_ = expected_next_capture_time;
    return true;
  }

} // namespace nao_camera

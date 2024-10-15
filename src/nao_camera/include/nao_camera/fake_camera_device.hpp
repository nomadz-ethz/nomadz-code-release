#pragma once

#include <chrono>

#include "nao_camera/base_camera_device.hpp"

namespace nao_camera {

  class FakeCameraDevice : public BaseCameraDevice {
  public:
    FakeCameraDevice(unsigned int width, unsigned int height, unsigned int fps);

    sensor_msgs::msg::Image::UniquePtr capture(int timeout) override;

    bool capture(sensor_msgs::msg::Image& image_msg, int timeout) override;

    std::vector<Control> getControls() const override { return {}; }

    int getControlValue([[maybe_unused]] unsigned int id) override { return 0; }

    bool setControlValue([[maybe_unused]] unsigned int id, [[maybe_unused]] int value) override { return true; }

  private:
    const uint16_t width_;
    const uint16_t height_;
    const uint16_t fps_;

    const uint16_t step_ = width_ * 2;
    const uint32_t image_data_size_ = width_ * step_;

    // init current time to null

    std::chrono::steady_clock::time_point last_capture_time_;
  };

} // namespace nao_camera

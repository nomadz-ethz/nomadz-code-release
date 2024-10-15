#pragma once

#include <vector>
#include <atomic>
#include <memory>

#include "nao_camera/base_camera_device.hpp"
#include "nao_camera/base_v4l2_device_interface.hpp"
#include "nao_camera/control.hpp"

namespace nao_camera {

  class NaoCameraDevice : public BaseCameraDevice {
  public:
    struct Settings {
      uint16_t width;
      uint16_t height;
      bool flip;
      uint16_t fps;
    };

    NaoCameraDevice(BaseV4l2DeviceInterface& device_interface, Settings settings);

    ~NaoCameraDevice() override;

    sensor_msgs::msg::Image::UniquePtr capture(int timeout) override;

    [[nodiscard]] bool capture(sensor_msgs::msg::Image& image_msg, int timeout) override;

    std::vector<Control> getControls() const override { return controls_; }

    int getControlValue(unsigned int id) override;

    bool setControlValue(unsigned int id, int value) override;

  private:
    bool start();

    bool stop();

    bool initMemoryMapping();

    static constexpr unsigned int NUM_V4L2_BUFFERS = 3;

    BaseV4l2DeviceInterface& v4l2_device_interface_;

    Settings settings_;

    std::vector<ImageBuffer> image_buffers_;

    std::vector<Control> controls_;
  };

} // namespace nao_camera

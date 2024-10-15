#pragma once

#include <cstdint>
#include <vector>

#include <sensor_msgs/msg/image.hpp>

#include "nao_camera/control.hpp"

namespace nao_camera {

  class BaseCameraDevice {
  public:
    virtual ~BaseCameraDevice() = default;

    virtual sensor_msgs::msg::Image::UniquePtr capture(int timeout) = 0;

    virtual bool capture(sensor_msgs::msg::Image& image_msg, int timeout) = 0;

    virtual std::vector<Control> getControls() const = 0;

    virtual int getControlValue(unsigned int id) = 0;

    virtual bool setControlValue(unsigned int id, int value) = 0;
  };

} // namespace nao_camera

#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <string>

#include "nao_camera/control.hpp"
#include "nao_camera/base_v4l2_device_interface.hpp"

namespace nao_camera {

  class V4l2DeviceInterface : public BaseV4l2DeviceInterface {
  public:
    explicit V4l2DeviceInterface(const std::string& device);

    ~V4l2DeviceInterface() override;

    int setImageFormat(uint32_t width, uint32_t height) override;

    int setStreamParameters(uint32_t fps) override;

    int setUvcExtensionUnit(uint8_t unit, uint8_t control, uint16_t size, uint8_t* data) override;

    int queueBuffer(uint32_t index) override;

    int dequeBuffer(uint32_t& index, timeval& timestamp) override;

    int startStream() override;

    int stopStream() override;

    PollResult poll(int timeout) override;

    int getControlValue(uint32_t id, int32_t& value) override;

    int setControlValue(uint32_t id, int32_t value) override;

    int requestBuffers(uint32_t& num_buffers) override;

    bool mmapBuffer(uint32_t index, ImageBuffer& buffer) override;

    bool munmapBuffer(ImageBuffer& buffer) override;

    std::vector<Control> getControls() const override;

  private:
    int fd_;
  };

} // namespace nao_camera

#pragma once

#include <bits/types/struct_timeval.h>

#include <cstdint>
#include <cstddef>
#include <vector>

#include "nao_camera/control.hpp"

namespace nao_camera {

  struct ImageBuffer {
    unsigned index;
    size_t length;
    uint8_t* data;
  };

  enum class PollResult : uint8_t {
    FAILED = 0,
    OK = 1,
    TIMEOUT = 2,
    ERROR = 3,
  };

  class BaseV4l2DeviceInterface {
  public:
    virtual ~BaseV4l2DeviceInterface() = default;

    virtual int setImageFormat(uint32_t width, uint32_t height) = 0;

    virtual int setStreamParameters(uint32_t fps) = 0;

    virtual int setUvcExtensionUnit(uint8_t unit, uint8_t control, uint16_t size, uint8_t* data) = 0;

    virtual int queueBuffer(uint32_t index) = 0;

    virtual int dequeBuffer(uint32_t& index, timeval& timestamp) = 0;

    virtual int startStream() = 0;

    virtual int stopStream() = 0;

    virtual PollResult poll(int timeout) = 0;

    virtual int getControlValue(uint32_t id, int32_t& value) = 0;

    virtual int setControlValue(uint32_t id, int32_t value) = 0;

    virtual int requestBuffers(uint32_t& num_buffers) = 0;

    virtual bool mmapBuffer(uint32_t index, ImageBuffer& buffer) = 0;

    virtual bool munmapBuffer(ImageBuffer& buffer) = 0;

    virtual std::vector<Control> getControls() const = 0;
  };

} // namespace nao_camera

#include "nao_camera/nao_camera_device.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>

#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <iostream>

#include <sensor_msgs/image_encodings.hpp>

#include "nao_camera/exception.hpp"

namespace nao_camera {

  NaoCameraDevice::NaoCameraDevice(BaseV4l2DeviceInterface& device_interface, Settings settings)
      : v4l2_device_interface_(device_interface), settings_(settings) {

    if (v4l2_device_interface_.setImageFormat(settings_.width, settings_.height) == -1) {
      throw NaoCameraDeviceError("Failed to set image format");
    }

    if (v4l2_device_interface_.setStreamParameters(settings_.fps) == -1) {
      throw NaoCameraDeviceError("Failed to set required framerate");
    }

    // set flip
    std::array<uint8_t, 2> data = {static_cast<uint8_t>(settings_.flip), static_cast<uint8_t>(settings_.flip)};
    if (v4l2_device_interface_.setUvcExtensionUnit(3, 12, 2, data.data()) == -1 ||
        v4l2_device_interface_.setUvcExtensionUnit(3, 13, 2, data.data()) == -1) {
      throw NaoCameraDeviceError("Failed to set extension unit settings!");
    }

    if (!start()) {
      throw NaoCameraDeviceError("Failed to start streaming!");
    }

    controls_ = v4l2_device_interface_.getControls();
  }

  NaoCameraDevice::~NaoCameraDevice() {
    stop();
  }

  bool NaoCameraDevice::start() {

    if (!initMemoryMapping()) {
      return false;
    }

    // Queue the buffers
    for (const auto& buffer : image_buffers_) {
      if (v4l2_device_interface_.queueBuffer(buffer.index) == -1) {
        std::cerr << "Failed to queue buffer " << buffer.index << " before capture start.\n";
        return false;
      }
    }

    if (v4l2_device_interface_.startStream() == -1) {
      std::cerr << "Failed to start video capture.\n";
      return false;
    }

    return true;
  }

  bool NaoCameraDevice::stop() {

    bool success = true;

    if (v4l2_device_interface_.stopStream() == -1) {
      std::cerr << "Failed to stop video capture.\n";
      success = false;
    }

    // De-initialize buffers
    for (const auto& buffer : image_buffers_) {
      if (!v4l2_device_interface_.munmapBuffer(image_buffers_[buffer.index])) {
        std::cerr << "Failed to unmap buffer " << buffer.index << " after capture stop.\n";
        success = false;
      }
    }

    image_buffers_.clear();

    // Free all buffers
    uint32_t num_requested_buffers = 0;
    if (v4l2_device_interface_.requestBuffers(num_requested_buffers) == -1) {
      std::cerr << "Failed to free image buffers after capture stop.\n";
      success = false;
    }

    return success;
  }

  sensor_msgs::msg::Image::UniquePtr NaoCameraDevice::capture(int timeout) {
    auto image_msg = std::make_unique<sensor_msgs::msg::Image>();

    if (!capture(*image_msg, timeout)) {
      return nullptr;
    }

    return image_msg;
  }

  bool NaoCameraDevice::capture(sensor_msgs::msg::Image& image_msg, int timeout) {

    // poll camera device
    switch (v4l2_device_interface_.poll(timeout)) {
    case PollResult::FAILED:
      std::cerr << "Polling camera device failed.\n";
      return false;
    case PollResult::TIMEOUT:
      std::cerr << timeout << " ms passed and there's still no image to read from the camera. Capture aborted.\n";
      return false;
    case PollResult::ERROR:
      std::cerr << "Polling camera device failed.\n";
      return false;
    default:
      break;
    }

    // Dequeue buffer with new image - this call is blocking if no new image is available
    uint32_t dequeued_buffer_index;
    timeval timestamp;
    if (v4l2_device_interface_.dequeBuffer(dequeued_buffer_index, timestamp) == -1) {
      std::cerr << "Failed to dequeue buffer.\n";
      return false;
    }

    if (dequeued_buffer_index >= image_buffers_.size()) {
      throw NaoCameraDeviceError("Dequeued buffer index out of range");
    }

    const uint8_t bytes_per_pixel = 2;
    const uint16_t step = settings_.width * bytes_per_pixel;
    // memcpy the data from the dequeued buffer to the output vector
    image_msg.data.resize(static_cast<std::size_t>(settings_.height * step));
    std::copy(image_buffers_[dequeued_buffer_index].data,
              image_buffers_[dequeued_buffer_index].data + image_msg.data.size(),
              image_msg.data.begin());

    // Requeue buffer to be reused for new captures
    if (v4l2_device_interface_.queueBuffer(dequeued_buffer_index) == -1) {
      std::cerr << "Failed to re-queue buffer.\n";
      return false;
    }

    image_msg.header.stamp.sec = static_cast<int32_t>(timestamp.tv_sec);
    image_msg.header.stamp.nanosec = timestamp.tv_usec * 1000;
    image_msg.height = settings_.height;
    image_msg.width = settings_.width;
    image_msg.encoding = sensor_msgs::image_encodings::YUV422_YUY2;
    image_msg.step = step;

    return true;
  }

  int NaoCameraDevice::getControlValue(unsigned int id) {

    auto control_it = std::find_if(controls_.begin(), controls_.end(), [id](const Control& c) { return c.id == id; });
    if (control_it == controls_.end()) {
      std::cerr << "Control " << id << " not found.\n";
      return 0;
    }

    int32_t value;
    if (v4l2_device_interface_.getControlValue(id, value) == -1) {
      std::cerr << "Failed getting value for control " << control_it->name << ".\n";
      return 0;
    }
    return value;
  }

  bool NaoCameraDevice::setControlValue(unsigned int id, int value) {
    auto control_it = std::find_if(controls_.begin(), controls_.end(), [id](const Control& c) { return c.id == id; });
    if (control_it == controls_.end()) {
      std::cerr << "Control " << id << " not found.\n";
      return false;
    }

    if (control_it->inactive) {
      std::cerr << "Control " << control_it->name << " is inactive.\n";
      return false;
    }

    return v4l2_device_interface_.setControlValue(id, value) != -1;
  }

  bool NaoCameraDevice::initMemoryMapping() {

    uint32_t num_requested_buffers = NUM_V4L2_BUFFERS;
    if (v4l2_device_interface_.requestBuffers(num_requested_buffers) == -1) {
      std::cerr << "Failed to request frame buffers.\n";
      return false;
    }

    if (num_requested_buffers < NUM_V4L2_BUFFERS) {
      std::cerr << "Failed to request " << NUM_V4L2_BUFFERS << " buffers.\n";
      return false;
    }

    image_buffers_ = std::vector<ImageBuffer>(num_requested_buffers);

    for (auto i = 0U; i < num_requested_buffers; ++i) {
      if (!v4l2_device_interface_.mmapBuffer(i, image_buffers_[i])) {
        std::cerr << "Failed to map frame buffer\n";
        return false;
      }
    }

    return true;
  }

} // namespace nao_camera

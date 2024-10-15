#include "nao_camera/v4l2_device_interface.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>

#include <limits>

#include "nao_camera/exception.hpp"

namespace nao_camera {

  V4l2DeviceInterface::V4l2DeviceInterface(const std::string& device) {
    // open device
    fd_ = ::open(device.c_str(), O_RDWR);
    if (fd_ < 0) {
      throw V4l2DeviceInterfaceError("Failed to open device");
    }

    // Add delay between opening and using camera device
    // This is required to allow the proper initialization of the camera device
    usleep(30000);
  }

  V4l2DeviceInterface::~V4l2DeviceInterface() {
    ::close(fd_);
  }

  int V4l2DeviceInterface::setImageFormat(uint32_t width, uint32_t height) {
    struct v4l2_format img_fmt_params {};
    img_fmt_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    img_fmt_params.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    img_fmt_params.fmt.pix.field = V4L2_FIELD_NONE;
    img_fmt_params.fmt.pix.width = width;
    img_fmt_params.fmt.pix.height = height;
    return ioctl(fd_, VIDIOC_S_FMT, &img_fmt_params);
  }

  int V4l2DeviceInterface::setStreamParameters(uint32_t fps) {
    struct v4l2_streamparm stream_params {};
    stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    stream_params.parm.capture.timeperframe.numerator = 1;
    stream_params.parm.capture.timeperframe.denominator = fps;
    return ioctl(fd_, VIDIOC_S_PARM, &stream_params);
  }

  int V4l2DeviceInterface::setUvcExtensionUnit(uint8_t unit, uint8_t control, uint16_t size, uint8_t* data) {
    struct uvc_xu_control_query query;
    query.unit = unit;
    query.selector = control;
    query.query = UVC_SET_CUR;
    query.size = size;
    query.data = data;
    return ioctl(fd_, UVCIOC_CTRL_QUERY, &query);
  }

  int V4l2DeviceInterface::queueBuffer(uint32_t index) {
    struct v4l2_buffer buffer {};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = index;
    return ioctl(fd_, VIDIOC_QBUF, &buffer);
  }

  int V4l2DeviceInterface::dequeBuffer(uint32_t& index, timeval& timestamp) {
    struct v4l2_buffer buffer {};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    const int ret = ioctl(fd_, VIDIOC_DQBUF, &buffer);
    if (ret != -1) {
      index = buffer.index;
      timestamp = buffer.timestamp;
    } else {
      index = std::numeric_limits<uint32_t>::max();
    }
    return ret;
  }

  int V4l2DeviceInterface::startStream() {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    return ioctl(fd_, VIDIOC_STREAMON, &type);
  }

  int V4l2DeviceInterface::stopStream() {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    return ioctl(fd_, VIDIOC_STREAMOFF, &type);
  }

  PollResult V4l2DeviceInterface::poll(int timeout) {
    struct pollfd poll_fd {};
    poll_fd.fd = fd_;
    poll_fd.events = POLLIN | POLLPRI;
    poll_fd.revents = 0;
    const int ret = ::poll(&poll_fd, 1, timeout);

    if (ret < 0) {
      return PollResult::FAILED;
    }

    if (ret == 0) {
      return PollResult::TIMEOUT;
    }

    if ((poll_fd.revents & (POLLERR | POLLNVAL)) != 0) {
      return PollResult::ERROR;
    }

    return PollResult::OK;
  }

  int V4l2DeviceInterface::getControlValue(uint32_t id, int32_t& value) {
    struct v4l2_control control {};
    control.id = id;
    const int ret = ioctl(fd_, VIDIOC_G_CTRL, &control);
    value = control.value;
    return ret;
  }

  int V4l2DeviceInterface::setControlValue(uint32_t id, int32_t value) {
    struct v4l2_control control {};
    control.id = id;
    control.value = value;
    return ioctl(fd_, VIDIOC_S_CTRL, &control);
  }

  int V4l2DeviceInterface::requestBuffers(uint32_t& num_buffers) {
    struct v4l2_requestbuffers req_buffers {};
    req_buffers.count = num_buffers;
    req_buffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req_buffers.memory = V4L2_MEMORY_MMAP;
    const int ret = ioctl(fd_, VIDIOC_REQBUFS, &req_buffers);
    num_buffers = req_buffers.count;
    return ret;
  }

  bool V4l2DeviceInterface::mmapBuffer(uint32_t index, ImageBuffer& buffer) {
    struct v4l2_buffer buf {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;
    const int ret = ioctl(fd_, VIDIOC_QUERYBUF, &buf);
    if (ret == -1) {
      return false;
    }

    buffer.index = index;
    buffer.length = buf.length;
    buffer.data =
      static_cast<unsigned char*>(mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset));
    return buffer.data != MAP_FAILED;
  }

  bool V4l2DeviceInterface::munmapBuffer(ImageBuffer& buffer) {
    const int ret = munmap(buffer.data, buffer.length);
    return ret != -1;
  }

  std::vector<Control> V4l2DeviceInterface::getControls() const {
    std::vector<Control> controls;

    auto queryctrl = v4l2_queryctrl{};
    queryctrl.id = V4L2_CID_USER_CLASS | V4L2_CTRL_FLAG_NEXT_CTRL;

    while (ioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl) == 0) {
      // Ignore disabled controls
      if ((queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) != 0U) {
        continue;
      }

      auto menu_items = std::map<int, std::string>{};
      if (queryctrl.type == static_cast<unsigned int>(ControlType::MENU)) {
        auto querymenu = v4l2_querymenu{};
        querymenu.id = queryctrl.id;

        // Query all enum values
        for (auto i = queryctrl.minimum; i <= queryctrl.maximum; i++) {
          querymenu.index = i;
          if (ioctl(fd_, VIDIOC_QUERYMENU, &querymenu) == 0) {
            menu_items[i] = reinterpret_cast<const char*>(querymenu.name);
          }
        }
      }

      auto control = Control{};
      control.id = queryctrl.id;
      control.name = std::string{reinterpret_cast<char*>(queryctrl.name)};
      control.type = static_cast<ControlType>(queryctrl.type);
      control.minimum = queryctrl.minimum;
      control.maximum = queryctrl.maximum;
      control.default_value = queryctrl.default_value;
      control.menu_items = std::move(menu_items);
      control.inactive = (queryctrl.flags & V4L2_CTRL_FLAG_INACTIVE) != 0;

      controls.push_back(control);

      // Get ready to query next item
      queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }

    return controls;
  }

} // namespace nao_camera

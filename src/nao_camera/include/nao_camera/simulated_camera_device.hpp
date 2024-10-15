#pragma once

#include <istream>
#include <vector>
#include <array>

#include <boost/asio.hpp>

#include "nao_camera/base_camera_device.hpp"

namespace nao_camera {

  class SimulatedCameraDevice : public BaseCameraDevice {
  public:
    SimulatedCameraDevice(const std::string& addr, int port);

    sensor_msgs::msg::Image::UniquePtr capture(int timeout) override;

    bool capture(sensor_msgs::msg::Image& image_msg, int timeout) override;

    std::vector<Control> getControls() const override { return {}; }

    int getControlValue([[maybe_unused]] unsigned int id) override { return 0; }

    bool setControlValue([[maybe_unused]] unsigned int id, [[maybe_unused]] int value) override { return true; }

  private:
    static constexpr const char* MAGIC_VALUE = "wbimage";

    struct Header {
      // NOLINTNEXTLINE(modernize-avoid-c-arrays)
      char magic_value[8]; // "wbimage"
      uint16_t tick;
      uint8_t cam_id;
      uint8_t bytes_per_pixel;
      uint16_t width;
      uint16_t height;
    };

    static_assert(sizeof(Header) == 16, "Header size is not 16 bytes");

    static void receiveHeader(boost::asio::streambuf& receive_buffer, Header& header) {
      std::istream is(&receive_buffer);
      is.read(reinterpret_cast<char*>(&header), sizeof(header));
    }

    static void receiveImage(boost::asio::streambuf& receive_buffer, std::vector<uint8_t>& image) {
      std::istream is(&receive_buffer);
      is.read(reinterpret_cast<char*>(image.data()), image.size()); // NOLINT(bugprone-narrowing-conversions)
    }

    boost::asio::io_service io_service_;
    boost::asio::ip::tcp::socket socket_;
  };

} // namespace nao_camera

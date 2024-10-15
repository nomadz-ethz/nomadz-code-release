#include "nao_camera/camera_device_factory.hpp"

#include "nao_camera/nao_camera_device.hpp"
#include "nao_camera/v4l2_device_interface.hpp"
#ifndef TARGET_ROBOT
#include "nao_camera/simulated_camera_device.hpp"
#include "nao_camera/fake_camera_device.hpp"
#endif

namespace nao_camera {
  enum class DeviceType : std::uint8_t { NAO, SIMULATED, FAKE };

  DeviceType toDeviceType(const std::string& device_type) {

    static const std::unordered_map<std::string, DeviceType> device_type_map = {
      {"nao", DeviceType::NAO},
      {"simulated", DeviceType::SIMULATED},
      {"fake", DeviceType::FAKE},
    };

    const auto it = device_type_map.find(device_type);

    if (it == device_type_map.end()) {
      throw std::runtime_error("Invalid device type: " + device_type);
    }

    return it->second;
  }

  std::shared_ptr<BaseCameraDevice> createCameraDeviceFromParameters(const Parameters& params) {
    const DeviceType type = toDeviceType(params.getDeviceType());

    switch (type) {
    case DeviceType::NAO: {
      // Since NaoCameraDevice takes a reference to a BaseV4l2DeviceInterface, we use a holder to wrap
      // an instance of NaoCameraDevice and a V4l2DeviceInterface  then we use the aliasing constructor of
      // std::shared_ptr to create a shared pointer to an instance of NaoCameraDevice which points to
      // the NaoCameraDevice instance inside the holder, but whose lifetime is controlled by the holder.
      // This technique is described in this talk: https://youtu.be/l6Y9PqyK1Mc?si=p7kigDmv2ArU5sY6&t=2131
      struct Holder {
        Holder(const std::string& video_device, const NaoCameraDevice::Settings& settings)
            : v4l2_device_interface(video_device), device(v4l2_device_interface, settings) {}
        V4l2DeviceInterface v4l2_device_interface; // NOLINT(misc-non-private-member-variables-in-classes)
        NaoCameraDevice device;                    // NOLINT(misc-non-private-member-variables-in-classes)
      };
      const NaoCameraDevice::Settings settings = {static_cast<uint16_t>(params.getImageSize()[0]),
                                                  static_cast<uint16_t>(params.getImageSize()[1]),
                                                  params.getIsFlipped(),
                                                  static_cast<uint16_t>(params.getFrameRate())};
      auto holder = std::make_shared<Holder>(params.getVideoDevice(), settings);
      return std::shared_ptr<NaoCameraDevice>(holder, &holder->device);
    }
#ifndef TARGET_ROBOT
    case DeviceType::SIMULATED: {
      const std::string video_device = params.getVideoDevice();
      const std::size_t pos = video_device.find(':');
      if (pos == std::string::npos) {
        throw std::runtime_error("Invalid video device for simulated camera: " + video_device);
      }
      const std::string ip_address = video_device.substr(0, pos);
      const std::string port = video_device.substr(pos + 1);
      return std::make_shared<SimulatedCameraDevice>(ip_address, std::stoi(port));
    }
    case DeviceType::FAKE: {
      return std::make_shared<FakeCameraDevice>(static_cast<unsigned int>(params.getImageSize()[0]),
                                                static_cast<unsigned int>(params.getImageSize()[1]),
                                                static_cast<unsigned int>(params.getFrameRate()));
    }
#endif
    default:
      throw std::runtime_error("Invalid device type: " + params.getDeviceType());
    }
  }

} // namespace nao_camera

#pragma once

#include <memory>

#include "nao_camera/parameters.hpp"
#include "nao_camera/base_camera_device.hpp"

namespace nao_camera {
  std::shared_ptr<BaseCameraDevice> createCameraDeviceFromParameters(const Parameters& params);
} // namespace nao_camera

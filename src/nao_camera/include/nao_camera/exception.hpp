#pragma once

#include <exception>
#include <string>

namespace nao_camera {
  class V4l2DeviceInterfaceError : public std::exception {
  public:
    explicit V4l2DeviceInterfaceError(std::string message) : message_(std::move(message)) {}

    const char* what() const noexcept override { return message_.c_str(); }

  private:
    std::string message_;
  };

  class NaoCameraDeviceError : public std::exception {

  public:
    explicit NaoCameraDeviceError(std::string message) : message_(std::move(message)) {}

    const char* what() const noexcept override { return message_.c_str(); }

  private:
    std::string message_;
  };

  class NaoCameraParameterError : public std::exception {
  public:
    explicit NaoCameraParameterError(std::string message) : message_(std::move(message)) {}
    const char* what() const noexcept override { return message_.c_str(); }

  private:
    std::string message_;
  };
} // namespace nao_camera

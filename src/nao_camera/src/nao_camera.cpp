#include "nao_camera/nao_camera.hpp"

#include <algorithm>
#include <cstring>

#include <sensor_msgs/image_encodings.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/logging.hpp>

#include "nao_camera/control.hpp"
#include "nao_camera/camera_device_factory.hpp"
#include "nao_camera/exception.hpp"

namespace nao_camera {

  NaoCamera::NaoCamera(const std::string& node_name, const rclcpp::NodeOptions& options)
      : Node(node_name, options), parameters_{get_node_parameters_interface(),
                                              get_node_topics_interface(),
                                              get_node_logging_interface()},
        private_subnode_{create_sub_node(get_name())},
        // here we pass the private subnode so that set_camera_info is in the right namespace
        camera_info_manager_{private_subnode_.get(), get_name()} {

    parameters_.declareStaticParameters();

    nao_camera_device_ = createCameraDeviceFromParameters(parameters_);

    // Add delay between opening and using camera device
    // FIXME(albanesg): this is done in B-Human, not sure if it's necessary
    rclcpp::sleep_for(std::chrono::microseconds(3000));

    parameters_.declareControlParameters(*nao_camera_device_);

    applyParameters();

    // NOLINTNEXTLINE(performance-unnecessary-value-param)
    parameters_.setParameterChangedCallback([this](rclcpp::Parameter parameter) { handleParameter(parameter); });

    // setup camera info
    if (!camera_info_manager_.validateURL(parameters_.getCameraInfoUrl())) {
      throw NaoCameraParameterError("Invalid camera info URL: " + parameters_.getCameraInfoUrl());
    }
    if (!camera_info_manager_.loadCameraInfo(parameters_.getCameraInfoUrl())) {
      throw NaoCameraParameterError("Failed to load camera info from URL: " + parameters_.getCameraInfoUrl());
    }

    // setup image publishing
    if (options.use_intra_process_comms()) {
      for (std::size_t i = 0; i < NUM_REUSABLE_IMAGE_MSGS; ++i) {
        image_msg_queue_.push(std::make_unique<ImageMsgT>());
      }
      image_pub_ = create_publisher<ImageMsgT>(RAW_IMAGE_TOPIC, 1);
      processed_image_sub_ = create_subscription<ImageMsgT>(
        PROCESSED_IMAGE_TOPIC, 1, [this](ImageMsgT::UniquePtr msg) { processedImageCallback(std::move(msg)); });
      camera_info_pub_ = create_publisher<CameraInfoMsgT>(CAMERA_INFO_TOPIC, 1);
    } else {
      camera_pub_ = image_transport::create_camera_publisher(this, RAW_IMAGE_TOPIC);
    }

    // start capture and publish thread
    if (options.use_intra_process_comms()) {
      image_publish_thread_ = std::thread([this]() {
        // NOLINTNEXTLINE(clang-analyzer-cplusplus.Move)
        cyclicCaptureAndPublishLoop();
      });
    } else {
      image_publish_thread_ = std::thread([this]() { captureAndPublishLoop(); });
    }
  }

  NaoCamera::NaoCamera(const rclcpp::NodeOptions& options) : NaoCamera("nao_camera", options) {}

  NaoCamera::~NaoCamera() {
    cancelled_.store(true);
    if (image_publish_thread_.joinable()) {
      image_publish_thread_.join();
    }
  }

  void NaoCamera::resetCameraDevice() {
    if (nao_camera_device_) {
      RCLCPP_INFO(this->get_logger(), "Stopping camera device.");
      nao_camera_device_ = nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "Starting camera device.");
    nao_camera_device_ = createCameraDeviceFromParameters(parameters_);
    applyParameters();
    RCLCPP_INFO(this->get_logger(), "Camera device restarted.");
  }

  void NaoCamera::applyParameters() {
    // Control parameters
    auto control_parameters = parameters_.getControlParameters();
    for (auto const& param : control_parameters) {
      auto control_id = parameters_.getControlId(param);

      switch (param.get_type()) {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        if (static_cast<bool>(nao_camera_device_->getControlValue(control_id)) == param.as_bool()) {
          continue;
        }
        nao_camera_device_->setControlValue(control_id, static_cast<int>(param.as_bool()));
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        if (nao_camera_device_->getControlValue(control_id) == param.as_int()) {
          continue;
        }
        nao_camera_device_->setControlValue(control_id, static_cast<int>(param.as_int()));
        break;
      default:
        RCLCPP_WARN(get_logger(),
                    "Control parameter type not currently supported: %d, for parameter: %s",
                    unsigned(param.get_type()),
                    param.get_name().c_str());
      }
    }
  }

  void NaoCamera::processedImageCallback(sensor_msgs::msg::Image::UniquePtr msg) {
    const std::lock_guard<std::mutex> lock(image_msg_queue_mutex_);
    image_msg_queue_.push(std::move(msg));
    image_msg_queue_cv_.notify_one();
  }

  bool NaoCamera::handleParameter(rclcpp::Parameter const& param) {
    const std::string& name = param.get_name();
    if (parameters_.isControlParameter(param)) {
      const int control_id = parameters_.getControlId(param);
      switch (param.get_type()) {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        if (static_cast<bool>(nao_camera_device_->getControlValue(control_id)) == param.as_bool()) {
          RCLCPP_DEBUG(get_logger(), "Parameter %s already set at requested value: %d", name.c_str(), param.as_bool());
          return true;
        }
        return nao_camera_device_->setControlValue(control_id, static_cast<int>(param.as_bool()));
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        if (nao_camera_device_->getControlValue(control_id) == param.as_int()) {
          RCLCPP_DEBUG(get_logger(), "Parameter %s already set at requested value: %ld", name.c_str(), param.as_int());
          return true;
        }
        return nao_camera_device_->setControlValue(control_id, static_cast<int>(param.as_int()));
      default:
        RCLCPP_WARN(get_logger(),
                    "Control parameter type not currently supported: %s, for parameter: %s",
                    std::to_string(unsigned(param.get_type())).c_str(),
                    param.get_name().c_str());
      }
    }

    return false;
  }

  void NaoCamera::captureAndPublishLoop() {
    ImageMsgT::SharedPtr image_msg = std::make_shared<ImageMsgT>();
    CameraInfoMsgT::SharedPtr camera_info_msg = std::make_shared<CameraInfoMsgT>();
    while (rclcpp::ok() && !cancelled_.load()) {
      if (nao_camera_device_->capture(*image_msg, CAPTURE_TIMEOUT_MSEC)) {
        image_msg->header.frame_id = parameters_.getCameraFrameId();
        *camera_info_msg = camera_info_manager_.getCameraInfo();
        camera_info_msg->header = image_msg->header;
        camera_pub_.publish(*image_msg, *camera_info_msg);
      } else {
        resetCameraDevice();
      }
    }
  }

  void NaoCamera::cyclicCaptureAndPublishLoop() {
    const auto delay_until_next_capture = std::chrono::milliseconds(500 / parameters_.getFrameRate());
    while (rclcpp::ok() && !cancelled_.load()) {

      // wait for image message to be available until timeout
      std::unique_lock<std::mutex> lock(image_msg_queue_mutex_);
      const bool image_msg_available =
        image_msg_queue_cv_.wait_for(lock, delay_until_next_capture, [this]() { return !image_msg_queue_.empty(); });

      // if image message is available, pull it from the queue, fill it with the next image
      ImageMsgT::UniquePtr image_msg = nullptr;
      if (image_msg_available) {
        image_msg = std::move(image_msg_queue_.front());
        image_msg_queue_.pop();
        // now the queue has been popped, we can unlock the mutex
        lock.unlock();

        // fill the image message with the next image
        bool ok = nao_camera_device_->capture(*image_msg, CAPTURE_TIMEOUT_MSEC);

        // if the capture failed, requeue the image message
        if (!ok) {
          image_msg_queue_.push(std::move(image_msg));
          image_msg = nullptr;
        }
      }
      // if no image message is available, allow the device to allocate a new one
      else {
        // unlock the mutex immediately, as we are not going to use the image message queue
        lock.unlock();

        // allocate a new image message
        // throttle warning, only print every 2 seconds
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Image message queue empty, reallocating message.");
        image_msg = nao_camera_device_->capture(CAPTURE_TIMEOUT_MSEC);
      }

      // publish the image message
      if (image_msg) {
        image_msg->header.frame_id = parameters_.getCameraFrameId();
        CameraInfoMsgT::UniquePtr camera_info_msg = std::make_unique<CameraInfoMsgT>(camera_info_manager_.getCameraInfo());
        camera_info_msg->header = image_msg->header;
        camera_info_pub_->publish(std::move(camera_info_msg));
        image_pub_->publish(std::move(image_msg));
      }
      // if the image message is nullptr, the capture failed and we need to reset the camera device
      else {
        resetCameraDevice();
      }
    }
  }
} // namespace nao_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nao_camera::NaoCamera)

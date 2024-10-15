#pragma once

#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <atomic>
#include <string>

#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>

#include "nao_camera/base_camera_device.hpp"
#include "nao_camera/parameters.hpp"

namespace nao_camera {

  class NaoCamera : public rclcpp::Node {
    using ImageMsgT = sensor_msgs::msg::Image;
    using CameraInfoMsgT = sensor_msgs::msg::CameraInfo;

    static constexpr const char* RAW_IMAGE_TOPIC = "~/image";
    static constexpr const char* PROCESSED_IMAGE_TOPIC = "~/image_processed";
    static constexpr const char* CAMERA_INFO_TOPIC = "~/camera_info";

    static constexpr unsigned int CAPTURE_TIMEOUT_MSEC = 1000;
    static constexpr std::size_t NUM_REUSABLE_IMAGE_MSGS = 2;

  public:
    NaoCamera(const std::string& node_name, const rclcpp::NodeOptions& options);

    explicit NaoCamera(const rclcpp::NodeOptions& options);

    ~NaoCamera() override;

  private:
    void resetCameraDevice();

    void applyParameters();

    bool handleParameter(rclcpp::Parameter const& param);

    void processedImageCallback(ImageMsgT::UniquePtr msg);

    void captureAndPublishLoop();

    void cyclicCaptureAndPublishLoop();

    std::shared_ptr<BaseCameraDevice> nao_camera_device_{nullptr};

    Parameters parameters_;

    // required to set the proper namespace for the camera info manager
    rclcpp::Node::SharedPtr private_subnode_;

    camera_info_manager::CameraInfoManager camera_info_manager_;

    // camera publisher using image_transport for non-intra-process communication
    image_transport::CameraPublisher camera_pub_;

    // pub / sub pair for cyclic pipeline with intra-process communication
    // see https://docs.ros.org/en/humble/Tutorials/Demos/Intra-Process-Communication.html#the-cyclic-pipeline-demo
    rclcpp::Publisher<ImageMsgT>::SharedPtr image_pub_{nullptr};
    rclcpp::Publisher<CameraInfoMsgT>::SharedPtr camera_info_pub_{nullptr};
    rclcpp::Subscription<ImageMsgT>::SharedPtr processed_image_sub_{nullptr};

    std::queue<std::unique_ptr<ImageMsgT>> image_msg_queue_;
    std::mutex image_msg_queue_mutex_;
    std::condition_variable image_msg_queue_cv_;

    std::thread image_publish_thread_;
    std::atomic<bool> cancelled_{false};
  };

} // namespace nao_camera

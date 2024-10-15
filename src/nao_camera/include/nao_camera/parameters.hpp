#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>

#include "nao_camera/base_camera_device.hpp"

namespace rclcpp::node_interfaces {
  struct PostSetParametersCallbackHandle;
} // namespace rclcpp::node_interfaces

namespace nao_camera {
  class Parameters {
  public:
    Parameters(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface,
               rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
               rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface);

    /** Declare read-only parameters */
    void declareStaticParameters();

    /** Declare control parameters by querying device */
    void declareControlParameters(const BaseCameraDevice& device);

    /** Set callbacks for inspecting and applying parameter changes */
    void setParameterChangedCallback(std::function<void(rclcpp::Parameter)> callback);

    rclcpp::Parameter getParameter(std::string const& name) const { return parameters_interface_->get_parameter(name); }

    const rclcpp::ParameterValue& getParameterValue(std::string const& name) const {
      return parameters_interface_->get_parameter(name).get_parameter_value();
    }

    template <typename T> decltype(auto) getValue(std::string const& name) const {
      return parameters_interface_->get_parameter(name).get_value<T>();
    }

    // Static parameters getters
    std::string getVideoDevice() const { return getValue<std::string>("video_device"); }
    std::string getDeviceType() const { return getValue<std::string>("device_type"); };
    std::vector<int64_t> getImageSize() const {
      return getValue<std::vector<int64_t>>("image_size"); // NOLINT(bugprone-narrowing-conversions)
    }
    bool getIsFlipped() const { return getValue<bool>("is_flipped"); }
    int32_t getFrameRate() const {
      return getValue<int32_t>("frame_rate"); // NOLINT(bugprone-narrowing-conversions)
    }
    std::string getCameraFrameId() const { return getValue<std::string>("camera_frame_id"); }
    std::string getCameraInfoUrl() const { return getValue<std::string>("camera_info_url"); }

    std::vector<rclcpp::Parameter> getControlParameters() const {
      auto names = std::vector<std::string>{};
      std::transform(
        control_name_to_id_.begin(), control_name_to_id_.end(), std::back_inserter(names), [](auto kv) { return kv.first; });

      return parameters_interface_->get_parameters(names);
    }

    bool isControlParameter(rclcpp::Parameter const& parameter) const {
      return control_name_to_id_.find(parameter.get_name()) != control_name_to_id_.end();
    }
    int32_t getControlId(std::string const& name) const {
      return control_name_to_id_.at(name); // NOLINT(bugprone-narrowing-conversions)
    }
    int32_t getControlId(rclcpp::Parameter const& param) const { return getControlId(param.get_name()); }

  private:
    using NodeLoggingInterface = rclcpp::node_interfaces::NodeLoggingInterface;
    using NodeParametersInterface = rclcpp::node_interfaces::NodeParametersInterface;
    using NodeTopicsInterface = rclcpp::node_interfaces::NodeTopicsInterface;

    NodeLoggingInterface::SharedPtr logging_interface_;
    NodeParametersInterface::SharedPtr parameters_interface_;
    NodeTopicsInterface::SharedPtr topics_interface_;

    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> on_set_parameter_callback_handle_;
    std::shared_ptr<rclcpp::node_interfaces::PostSetParametersCallbackHandle> post_set_parameter_callback_handle_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

    std::function<void(rclcpp::Parameter)> parameter_changed_callback_;

    std::unordered_map<std::string, uint32_t> control_name_to_id_;

    inline static rcl_interfaces::msg::ParameterDescriptor
    makeDescriptor(std::string description, std::string additional_constraints, bool read_only) {
      auto parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
      parameter_descriptor.description = std::move(description);
      parameter_descriptor.additional_constraints = std::move(additional_constraints);
      parameter_descriptor.read_only = read_only;
      return parameter_descriptor;
    }

    template <typename T>
    void declareParameter(std::string const& name,
                          T value,
                          rcl_interfaces::msg::ParameterDescriptor const& parameter_descriptor) {
      auto parameter_value = rclcpp::ParameterValue{value};
      parameters_interface_->declare_parameter(name, parameter_value, parameter_descriptor);
    }

    template <typename T>
    void declareParameter(std::string const& name,
                          T value,
                          std::string const& description,
                          std::string const& additional_constraints,
                          bool read_only = false) {
      auto parameter_descriptor = makeDescriptor(description, additional_constraints, read_only);
      declareParameter(name, value, parameter_descriptor);
    }

    template <typename T>
    void declareParameter(std::string const& name, T value, std::string const& description, bool read_only = false) {
      declareParameter(name, value, description, "", read_only);
    }

    template <typename T>
    void setParameterChangedCallbackImpl(std::shared_ptr<T> /*parameters_interface*/,
                                         rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
                                         std::function<void(rclcpp::Parameter)> callback) {
      parameter_event_sub_ = rclcpp::AsyncParametersClient::on_parameter_event(
        topics_interface,
        // NOLINTNEXTLINE(performance-unnecessary-value-param)
        [this, callback = std::move(callback)](rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event) {
          if (event->node != topics_interface_->get_node_base_interface()->get_fully_qualified_name()) {
            return;
          }
          for (auto const& parameter : event->changed_parameters) {
            callback(parameters_interface_->get_parameter(parameter.name));
          }
        });
    }
  };
} // namespace nao_camera

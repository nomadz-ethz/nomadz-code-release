#include "nao_camera/parameters.hpp"

#include <string>

namespace nao_camera {

  Parameters::Parameters(NodeParametersInterface::SharedPtr parameters_interface,
                         NodeTopicsInterface::SharedPtr topics_interface,
                         NodeLoggingInterface::SharedPtr logging_interface)
      : logging_interface_{std::move(logging_interface)}, parameters_interface_{std::move(parameters_interface)},
        topics_interface_{std::move(topics_interface)} {}

  void Parameters::declareStaticParameters() {
    declareParameter("video_device", "/dev/video0", "Path to video device", true);
    declareParameter("device_type", "nao", "Type of device", true);
    declareParameter("image_size", std::vector<int64_t>{640, 480}, "Image size", true);
    declareParameter("frame_rate", 30, "Frame rate", true);
    declareParameter("is_flipped", false, "Whether the image is flipped", true);
    declareParameter("camera_frame_id", "camera", "Frame id inserted in published image", true);
    declareParameter("camera_info_url", "", "URL to camera info file", true);
  }

  void Parameters::declareControlParameters(const BaseCameraDevice& device) {
    // Helper to transform control name to parameter name
    // - makes all lower case
    // - removes ',', '(' and ')'
    // - replaces spaces with underscores
    auto to_param_name = [](std::string name) {
      std::transform(name.begin(), name.end(), name.begin(), ::tolower);
      name.erase(std::remove(name.begin(), name.end(), ','), name.end());
      name.erase(std::remove(name.begin(), name.end(), '('), name.end());
      name.erase(std::remove(name.begin(), name.end(), ')'), name.end());
      std::replace(name.begin(), name.end(), ' ', '_');
      return name;
    };

    for (auto const& c : device.getControls()) {
      auto name = to_param_name(c.name);
      auto descriptor = makeDescriptor(c.name, "", false);
      switch (c.type) {
      case ControlType::INT: {
        auto range = rcl_interfaces::msg::IntegerRange{};
        range.from_value = c.minimum;
        range.to_value = c.maximum;
        descriptor.integer_range.push_back(range);
        declareParameter<int64_t>(name, c.default_value, descriptor);
        break;
      }
      case ControlType::BOOL: {
        declareParameter<bool>(name, c.default_value != 0, descriptor);
        break;
      }
      case ControlType::MENU: {
        auto sstr = std::ostringstream{};
        for (auto const& o : c.menu_items) {
          sstr << o.first << " - " << o.second << ", ";
        }
        auto str = sstr.str();
        descriptor.additional_constraints = str.substr(0, str.size() - 2);
        declareParameter<int64_t>(name, c.default_value, descriptor);
        break;
      }
      default:
        RCLCPP_WARN(logging_interface_->get_logger(),
                    "Control type not currently supported: %s, for control: %s",
                    std::to_string(unsigned(c.type)).c_str(),
                    c.name.c_str());
        continue;
      }
      control_name_to_id_[name] = c.id;
    }
  }

  void Parameters::setParameterChangedCallback(std::function<void(rclcpp::Parameter)> callback) {
    // Callback for inspecting and validating changes
    on_set_parameter_callback_handle_ =
      parameters_interface_->add_on_set_parameters_callback([](std::vector<rclcpp::Parameter> const& /*parameters*/) {
        auto result = rcl_interfaces::msg::SetParametersResult{};
        result.successful = true;
        return result;
      });

    // Callback for actually applying changes
    setParameterChangedCallbackImpl(parameters_interface_, topics_interface_, callback);
  }

} // namespace nao_camera

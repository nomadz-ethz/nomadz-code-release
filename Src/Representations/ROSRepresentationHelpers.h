/**
 * @file ROSRepresentationHelpers.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

// Explicitly instantiate ROS helpers for modules as this is extremely memory-heavy and slow.
// FIXME: Improve compile-time penalty of instantiation.

#include "Core/Module/ROSModuleHelpers.h"
#include "Core/Module/ROSModuleHelpers.tpp"

namespace internal {
#ifdef ENABLE_ROS
  template <typename Representation>
  void ros_handle_in_repr(const std::string& repr_name, bool has_stream, rclcpp::Node* ros_node, Representation& repr) {
    if (ros_node == nullptr || !rclcpp::ok()) {
      return;
    }
    static std::mutex mutex;
    static rclcpp::Clock system_clock;
    const std::lock_guard<std::mutex> lock(mutex);
    static std::map<std::string, std::shared_ptr<rclcpp::Subscription<typename Representation::ROSDeclType>>> ros_subs;

    auto iter = ros_subs.find(ros_node->get_name());
    if (iter == ros_subs.end()) {
      iter =
        ros_subs.emplace(ros_node->get_name(), internal::create_ros_subscription<Representation>(ros_node, repr_name)).first;
    }
    if (iter->second != nullptr) {
      Representation ros_repr;
      if (internal::ros_take(iter->second.get(), ros_repr)) {
        if (Representation::ros_compatible) {
          if (has_stream) {
            internal::ros_print_diff<Representation>(ros_node, &repr, &ros_repr, repr_name);
          }
          repr = std::move(ros_repr);
        } else if (!has_stream) {
          OUTPUT_WARNING("EXTERNAL RECEIVE FOR NON-ROS MESSAGE " + repr_name);
        }
      } else {
        RCLCPP_DEBUG_THROTTLE(ros_node->get_logger(), system_clock, 1000, "NO MESSAGE RECEIVED FOR %s", repr_name.c_str());
      }
    } else {
      OUTPUT_WARNING("INVALID SUBSCRIBER FOR " + repr_name);
    }
  }

  template <typename Representation>
  void ros_handle_out_repr(const std::string& repr_name, bool has_stream, rclcpp::Node* ros_node, Representation& repr) {
    if (ros_node == nullptr || !rclcpp::ok()) {
      return;
    }
    static std::mutex mutex;
    const std::lock_guard<std::mutex> lock(mutex);
    static std::map<std::string, std::shared_ptr<rclcpp::Publisher<typename Representation::ROSDeclType>>> ros_pubs;

    auto iter = ros_pubs.find(ros_node->get_name());
    if (iter == ros_pubs.end()) {
      iter =
        ros_pubs.emplace(ros_node->get_name(), internal::create_ros_publisher<Representation>(ros_node, repr_name)).first;
    }
    if (iter->second != nullptr) {
      if (!has_stream && !Representation::ros_compatible) {
        OUTPUT_WARNING("EXTERNAL SEND FOR NON-ROS MESSAGE " + repr_name);
      }
      internal::ros_publish<Representation>(iter->second.get(), repr);
    } else {
      OUTPUT_WARNING("INVALID PUBLISHER FOR " + repr_name);
    }
  }
#else
  template <typename Representation>
  void ros_handle_in_repr(const std::string& repr_name, bool has_stream, rclcpp::Node* ros_node, Representation& repr) {}
  template <typename Representation>
  void ros_handle_out_repr(const std::string& repr_name, bool has_stream, rclcpp::Node* ros_node, Representation& repr) {}
#endif

} // namespace internal

// Create explicit instantiations of representations specific module functions
#define INSTANTIATE_ROS_HELPERS(representation)                                                                             \
  template void internal::ros_handle_in_repr<representation>(                                                               \
    const std::string& repr_name, bool has_stream, rclcpp::Node* ros_node, representation& repr);                           \
  template void internal::ros_handle_out_repr<representation const>(                                                        \
    const std::string& repr_name, bool has_stream, rclcpp::Node* ros_node, representation const& repr);

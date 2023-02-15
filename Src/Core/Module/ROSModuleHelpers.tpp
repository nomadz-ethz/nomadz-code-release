/**
 * @file ROSModuleHelpers.tpp
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <list>
#include <string>
#include <memory>
#include <mutex>
#include <map>

#include "ROSModuleHelpers.h"

#include "Core/System/BHAssert.h"
#include "Core/Debugging/Debugging.h"

#ifdef ENABLE_ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#endif

namespace internal {
#ifdef ENABLE_ROS
  inline rclcpp::QoS default_qos() { return rclcpp::QoS(rclcpp::KeepLast(10)); }
#endif

  template <typename T>
  std::shared_ptr<rclcpp::Publisher<typename T::ROSDeclType>> create_ros_publisher(void* node, std::string name) {
    OUTPUT_WARNING("CANNOT PUBLISH MESSAGE " + name);
    return nullptr;
  }
#ifdef ENABLE_ROS
  template <typename T, typename M>
  std::shared_ptr<rclcpp::Publisher<M>> create_ros_publisher(rclcpp::Node* ros_node, std::string name) {
    RCLCPP_DEBUG(ros_node->get_logger(), "CREATING PUBLISHER FOR %s", name.c_str());
    return ros_node->create_publisher<M>("representations/" + name, default_qos());
  }
#endif

  template <typename T>
  std::shared_ptr<rclcpp::Subscription<typename T::ROSDeclType>> create_ros_subscription(void* node, std::string name) {
    OUTPUT_WARNING("CANNOT SUBSCRIBE TO MESSAGE " + name);
    return nullptr;
  }
#ifdef ENABLE_ROS
  template <typename T, typename M>
  std::shared_ptr<rclcpp::Subscription<M>> create_ros_subscription(rclcpp::Node* ros_node, std::string name) {
    RCLCPP_DEBUG(ros_node->get_logger(), "CREATING SUBSCRIPTION TO %s", name.c_str());
    return ros_node->create_subscription<M>("representations/" + name, default_qos(), [](const M& msg) { ASSERT(false); });
  }
#endif

  template <typename T> void ros_publish(void*, T) {}
#ifdef ENABLE_ROS
  // Make a copy to ensure the data is bound to the base directly.
  template <typename T, typename> void ros_publish(rclcpp::Publisher<typename T::ROSType>* pub, T msg) {
    pub->publish(static_cast<typename T::ROSType>(msg));
  }
#endif

  template <typename T> bool ros_take(void*, const T&) { return false; }
#ifdef ENABLE_ROS
  template <typename T, typename> bool ros_take(rclcpp::Subscription<typename T::ROSType>* sub, T& msg) {
    rclcpp::MessageInfo msg_info;
    // The copy message is not complete, the wrapper types are not kept in sync with only ROS assignment.
    T msg_copy;
    bool ret = false;
    while (sub->take(msg_copy, msg_info)) {
      ret = true;
    }
    if (ret) {
      // Force assignment to fix field wrappers.
      msg = msg_copy;
    }
    return ret;
  }
#endif

  template <typename T> void ros_print_diff(void*, void*, void*, const std::string& rep) {}
#ifdef ENABLE_ROS
  template <typename T, typename>
  void
  ros_print_diff(rclcpp::Node* ros_node, const typename T::ROSType* msg1, const typename T::ROSType* msg2, std::string rep) {
    static rclcpp::Clock system_clock;
    if (*msg1 != *msg2) {
      RCLCPP_DEBUG_THROTTLE(ros_node->get_logger(),
                            system_clock,
                            1000,
                            "MESSAGE %s NOT EQUAL FOR %s",
                            rosidl_generator_traits::name<typename T::ROSType>(),
                            rep.c_str());
    }
  }
#endif

} // namespace internal

/**
 * @file ROSModuleHelpers.tpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
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
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rmw/qos_profiles.h>
#include <rcl/publisher.h>
#include <rcl/subscription.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#endif

namespace internal {
#ifdef ENABLE_ROS
  inline rclcpp::QoS default_qos() { return rclcpp::QoS(rclcpp::KeepLast(10)); }
#endif

  template <typename T> std::shared_ptr<rcl_publisher_t> create_ros_publisher(void* node, std::string name) {
    OUTPUT_WARNING("CANNOT PUBLISH MESSAGE " + name);
    return nullptr;
  }
#ifdef ENABLE_ROS
  template <typename T, typename M>
  std::shared_ptr<rcl_publisher_t> create_ros_publisher(rclcpp::Node* ros_node, std::string name) {
    RCLCPP_DEBUG(ros_node->get_logger(), "CREATING PUBLISHER FOR %s", name.c_str());

    auto pub = std::make_shared<rcl_publisher_t>(rcl_get_zero_initialized_publisher());
    rcl_publisher_options_t pub_opt = rcl_publisher_get_default_options();
    pub_opt.qos = default_qos().get_rmw_qos_profile();
    rcl_ret_t rc = rcl_publisher_init(pub.get(),
                                      ros_node->get_node_base_interface()->get_rcl_node_handle(),
                                      rosidl_typesupport_cpp::get_message_type_support_handle<M>(),
                                      ("representations/" + name).c_str(),
                                      &pub_opt);
    if (RCL_RET_OK != rc) {
      rcl_reset_error();
      return nullptr;
    }
    return pub;
  }
#endif

  template <typename T> std::shared_ptr<rcl_subscription_t> create_ros_subscription(void* node, std::string name) {
    OUTPUT_WARNING("CANNOT SUBSCRIBE TO MESSAGE " + name);
    return nullptr;
  }
#ifdef ENABLE_ROS
  template <typename T, typename M>
  std::shared_ptr<rcl_subscription_t> create_ros_subscription(rclcpp::Node* ros_node, std::string name) {
    RCLCPP_DEBUG(ros_node->get_logger(), "CREATING SUBSCRIPTION TO %s", name.c_str());

    auto sub = std::make_shared<rcl_subscription_t>(rcl_get_zero_initialized_subscription());
    rcl_subscription_options_t sub_opt = rcl_subscription_get_default_options();
    sub_opt.qos = default_qos().get_rmw_qos_profile();
    rcl_ret_t rc = rcl_subscription_init(sub.get(),
                                         ros_node->get_node_base_interface()->get_rcl_node_handle(),
                                         rosidl_typesupport_cpp::get_message_type_support_handle<M>(),
                                         ("representations/" + name).c_str(),
                                         &sub_opt);
    if (rc != RCL_RET_OK) {
      rcl_reset_error();
      return nullptr;
    }
    return sub;
  }
#endif

  template <typename T> void ros_publish(void*, T) {}
#ifdef ENABLE_ROS
  // Make a copy to ensure the data is bound to the base directly.
  template <typename T, typename M> void ros_publish(rcl_publisher_t* pub, T msg) {
    rcl_ret_t rc = rcl_publish(pub, static_cast<const void*>(&static_cast<const M&>(msg)), nullptr);
    if (RCL_RET_OK != rc) {
      rcl_reset_error();
      OUTPUT_WARNING("FAILED TO PUBLISH ROS MESSAGE");
    }
  }
#endif

  template <typename T> bool ros_take(void*, const T&) { return false; }
#ifdef ENABLE_ROS
  template <typename T, typename M> bool ros_take(rcl_subscription_t* sub, T& msg) {
    rmw_message_info_t msg_info;
    // The copy message is not complete, the wrapper types are not kept in sync with only ROS assignment.
    T msg_copy;
    bool ret = false;
    rcl_ret_t rc = RCL_RET_SUBSCRIPTION_TAKE_FAILED;
    while ((rc = rcl_take(sub, static_cast<void*>(&static_cast<M&>(msg_copy)), &msg_info, nullptr)) == RCL_RET_OK) {
      ret = true;
    }
    if (rc != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
      rcl_reset_error();
      OUTPUT_WARNING("FAILED TO TAKE ROS MESSAGE");
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

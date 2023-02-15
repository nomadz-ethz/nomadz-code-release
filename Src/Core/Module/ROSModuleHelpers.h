/**
 * @file ROSModuleHelpers.h
 *
 * Contains various helper functions for modules
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */
#pragma once

#include <list>
#include <string>
#include <memory>
#include <mutex>
#include <map>

#ifdef ENABLE_ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#else
namespace rclcpp {
  struct Node {
    const char* get_name() { return ""; }
    int* get_logger() { return nullptr; }
  };

  template <typename> struct Subscription {};
  template <typename> struct Publisher {};
  struct Clock {};

  inline bool ok() { return false; }
} // namespace rclcpp
#define RCLCPP_DEBUG_THROTTLE(logger, clock, ms, msg, ...)                                                                  \
  do {                                                                                                                      \
    (void)logger;                                                                                                           \
    (void)clock;                                                                                                            \
    OUTPUT_WARNING("ROS DEBUG: " msg);                                                                                      \
  } while (false);
#endif

namespace internal {
  template <typename T>
  std::shared_ptr<rclcpp::Publisher<typename T::ROSDeclType>> create_ros_publisher(void* node, std::string name);

#ifdef ENABLE_ROS
  template <typename T, typename M = typename T::ROSType>
  std::shared_ptr<rclcpp::Publisher<M>> create_ros_publisher(rclcpp::Node* ros_node, std::string name);
#endif

  template <typename T>
  std::shared_ptr<rclcpp::Subscription<typename T::ROSDeclType>> create_ros_subscription(void* node, std::string name);
#ifdef ENABLE_ROS
  template <typename T, typename M = typename T::ROSType>
  std::shared_ptr<rclcpp::Subscription<M>> create_ros_subscription(rclcpp::Node* ros_node, std::string name);
#endif

  // Make a copy to ensure the data is bound to the base directly.
  template <typename T> void ros_publish(void*, T);
#ifdef ENABLE_ROS
  template <typename T, typename = std::void_t<typename T::ROSType>>
  void ros_publish(rclcpp::Publisher<typename T::ROSType>* pub, T msg);
#endif

  template <typename T> bool ros_take(void*, const T&);
#ifdef ENABLE_ROS
  template <typename T, typename = std::void_t<typename T::ROSType>>
  bool ros_take(rclcpp::Subscription<typename T::ROSType>* sub, T& msg);
#endif

  template <typename T> void ros_print_diff(void*, void*, void*, const std::string& rep);
#ifdef ENABLE_ROS
  template <typename T, typename = std::void_t<typename T::ROSType>>
  void
  ros_print_diff(rclcpp::Node* ros_node, const typename T::ROSType* msg1, const typename T::ROSType* msg2, std::string rep);
#endif

  template <typename Representation>
  void ros_handle_in_repr(const std::string& repr_name, bool has_stream, rclcpp::Node* ros_node, Representation& repr);

  template <typename Representation>
  void ros_handle_out_repr(const std::string& repr_name, bool has_stream, rclcpp::Node* ros_node, Representation& repr);
} // namespace internal

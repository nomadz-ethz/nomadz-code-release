#pragma once

#include <array>
#include <algorithm>
#include <exception>
#include <functional>
#include <rclcpp/rclcpp.hpp>

#include "nao_lola_sensor_msgs/msg/joint_data.hpp"
#include "nao_lola_command_msgs/msg/joint_requests.hpp"
#include "nomadz_definitions/joint_indexes.hpp"
#include "nomadz_motion_control/joint_positions.hpp"
#include "nomadz_motion_control/joint_stiffnesses.hpp"

namespace nomadz_motion_control {

  struct JointRequests {
    JointRequests() = default;

    explicit JointRequests(const nao_lola_sensor_msgs::msg::JointData& msg) {
      for (unsigned int i = 0; i < nomadz_definitions::joint_indexes::NUM_JOINTS; ++i) {
        positions.values[i] = msg.positions[i];
        stiffnesses.values[i] = msg.stiffnesses[i];
      }
    }

    explicit JointRequests(const nao_lola_command_msgs::msg::JointRequests& msg) {
      for (unsigned int i = 0; i < nomadz_definitions::joint_indexes::NUM_JOINTS; ++i) {
        positions.values[i] = 0.0;
        stiffnesses.values[i] = 0.0;
      }
      for (size_t i = 0; i < msg.indexes.size(); ++i) {
        uint8_t index = msg.indexes[i];
        positions.values[index] = msg.positions[index];
        stiffnesses.values[i] = msg.stiffnesses[index];
      }
    }

    void mirror(const JointRequests& other) {
      positions.mirror(other.positions);
      stiffnesses.mirror(other.stiffnesses);
    }

    JointRequests mirror() const {
      JointRequests mirrored_joint_request;
      mirrored_joint_request.positions = positions.mirror();
      mirrored_joint_request.stiffnesses = stiffnesses.mirror();
      return mirrored_joint_request;
    }

    inline JointRequests operator*(float scalar) const {
      JointRequests joint_request_scaled;
      joint_request_scaled.positions = positions * scalar;
      joint_request_scaled.stiffnesses = stiffnesses * scalar;
      return joint_request_scaled;
    }

    /**
     * @brief sums up two jointrequests
     *
     * The positions and stiffnesses are summed for each joint while
     * a the resulting joint_ignore is a "bitwise and"
     */
    inline JointRequests operator+(const JointRequests& other) const {
      JointRequests joint_request_sum;
      joint_request_sum.positions = positions + other.positions;
      joint_request_sum.stiffnesses = stiffnesses + other.stiffnesses;
      // create interpolated mask by using AND
      std::transform(joint_ignore.begin(),
                     joint_ignore.end(),
                     other.joint_ignore.begin(),
                     joint_request_sum.joint_ignore.begin(),
                     std::logical_and<bool>());
      return joint_request_sum;
    }

    /**
     * @brief subtracts two jointrequests
     *
     * The positions and stiffnesses are subtracted for each joint while
     * a the resulting joint_ignore is a "bitwise and"
     */
    inline JointRequests operator-(const JointRequests& other) const {
      JointRequests joint_request_diff;
      joint_request_diff.positions = positions - other.positions;
      joint_request_diff.stiffnesses = stiffnesses - other.stiffnesses;
      // create interpolated mask by using AND
      std::transform(joint_ignore.begin(),
                     joint_ignore.end(),
                     other.joint_ignore.begin(),
                     joint_request_diff.joint_ignore.begin(),
                     std::logical_and<bool>());
      return joint_request_diff;
    }

    /**
     * @brief Combine two JointRequests
     *
     * Internally calls operator+ but also does some checks to make sure
     * the result is valid. I.e. do not combine requests where both JointRequests
     * define a request for the same joint.
     *
     * @param request_1 The first JointRequest which should be combined
     * @param request_2 The second JointRequest which should be combined
     * @return JointRequests is the combined request
     */
    static JointRequests combineJointRequests(const JointRequests& request_1, const JointRequests& request_2) {
      std::array<bool, nomadz_definitions::joint_indexes::NUM_JOINTS> out_arr;
      std::transform(request_1.joint_ignore.begin(),
                     request_1.joint_ignore.end(),
                     request_2.joint_ignore.begin(),
                     out_arr.begin(),
                     std::not_fn(std::logical_or<bool>()));

      if (std::any_of(out_arr.begin(), out_arr.end(), [](bool x) { return x; })) {
        throw std::invalid_argument(
          "Trying to call combineJointRequests on two JointRequest with incompatible joint_ignore");
      }

      JointRequests return_request = request_1;
      return return_request + request_2;
    }

    /**
     * @brief Prioritized combination of two JointRequests
     *
     * Sets the values for the second joint request and overwrites them
     * with the first joint request if both define to not ignore the joint.
     *
     * @param request_1 The prioritized JointRequest
     * @param request_2 The other JointRequest
     * @return JointRequests, the combined request
     */
    static JointRequests prioritizedCombination(const JointRequests& request_1, const JointRequests& request_2) {
      JointRequests out_joint_request;
      for (int i = 0; i < nomadz_definitions::joint_indexes::NUM_JOINTS; ++i) {
        out_joint_request.positions.values[i] =
          !request_2.joint_ignore[i] ? request_2.positions.values[i] : static_cast<float>(0.0);
        out_joint_request.stiffnesses.values[i] =
          !request_2.joint_ignore[i] ? request_2.stiffnesses.values[i] : static_cast<float>(0.0);
        out_joint_request.positions.values[i] =
          !request_1.joint_ignore[i] ? request_1.positions.values[i] : out_joint_request.positions.values[i];
        out_joint_request.stiffnesses.values[i] =
          !request_1.joint_ignore[i] ? request_1.stiffnesses.values[i] : out_joint_request.stiffnesses.values[i];
        out_joint_request.joint_ignore[i] = request_1.joint_ignore[i] && request_2.joint_ignore[i];
      }
      return out_joint_request;
    }

    /**
     * @brief Converts the struct to a nao_lola_command_msgs JointRequests message
     */
    nao_lola_command_msgs::msg::JointRequests toJointRequestsMsg(const rclcpp::Time& time_stamp) const {
      nao_lola_command_msgs::msg::JointRequests joint_requests_msg;
      joint_requests_msg.header.stamp = time_stamp;
      for (uint8_t i = 0; i < nomadz_definitions::joint_indexes::NUM_JOINTS; ++i) {
        if (joint_ignore[i]) {
          continue;
        }
        joint_requests_msg.indexes.push_back(i);
        joint_requests_msg.positions.push_back(positions.values[i]);
        joint_requests_msg.stiffnesses.push_back(static_cast<float>(stiffnesses.values[i]));
      }
      return joint_requests_msg;
    }

    /**
     * @brief populates members indicating which joints are ignored
     *
     * Specifically, the joint_requests position are set to zero
     * for ignored joints as specified in joint_ignore_
     */
    static void updateJointRequestWithMask(JointRequests& joint_requests) {
      for (int i = 0; i < nomadz_definitions::joint_indexes::NUM_JOINTS; ++i) {
        if (joint_requests.joint_ignore[i]) {
          // NOTE(@naefjo): We set the ignored joint values to 0. This allows us to use operator
          // overloading to do the motion combination.
          joint_requests.positions.values[i] = 0.0;
        }
      }
    }

    JointPositions positions{};     // NOLINT(misc-non-private-member-variables-in-classes)
    JointStiffnesses stiffnesses{}; // NOLINT(misc-non-private-member-variables-in-classes)

    // Which joints are considered ignored/are unset by this JointRequest
    std::array<bool, nomadz_definitions::joint_indexes::NUM_JOINTS>
      joint_ignore{}; // NOLINT(misc-non-private-member-variables-in-classes)
  };

  using JointData = JointRequests;
} // namespace nomadz_motion_control

#pragma once

#include <cstdint>
#include <rclcpp/rclcpp.hpp>

#include "nomadz_motion_control/motion_net_data.hpp"
#include "nomadz_motion_control/motion_base.hpp"
#include "nomadz_definitions/joint_indexes.hpp"
#include "nomadz_motion_control/joint_requests.hpp"
#include "nao_lola_sensor_msgs/msg/joint_data.hpp"
#include "nao_lola_command_msgs/msg/joint_requests.hpp"
#include "nomadz_motion_control_msgs/msg/special_action_request.hpp"
#include "nomadz_motion_control_msgs/special_action_request_enums.hpp"
#include "nomadz_motion_control_msgs/msg/motion_info.hpp"
#include "special_action_generator_parameters.hpp"

namespace nomadz_motion_control {
  class SpecialActionGenerator : public MotionBase {

  public:
    explicit SpecialActionGenerator(const rclcpp::NodeOptions& options);

  private:
    enum class SpecialActionMode { DEACTIVE, ACTIVE, FIRST } special_action_mode_;

    void updateJointRequests() override;
    void updateMotionInfoMsg() override;
    bool isLeavingPossible() override;
    void reset() override;

    bool getNextData(const short& special_action_type);
    void calculateJointRequest(const JointRequests& target_joint_request, JointRequests& joint_requests);

    /**
     * @brief populates private members indicating which joints are ignored or off
     *
     * Specifically, the joint_requests position are set to the current joint_datas
     * position value
     */
    void processSpecialJointValues(JointRequests& joint_requests);

    special_action_generator_parameters::ParamListener param_listener_;

    MotionNetData motion_net_data_;
    SpecialActionRequestMsgT current_special_action_request_msg_;

    JointRequests next_target_joint_requests_;
    short current_node_;
    bool interpolation_mode_;
    bool deshake_mode_;

    bool was_end_of_special_action_{false};
    bool special_action_finished_{false};

    int hardness_interpolation_counter_{0};
    int hardness_interpolation_length_{0};
    int data_repetition_length_{0};
    int data_repetition_counter_{0};

    short executed_special_action_type_;
  };
} // namespace nomadz_motion_control

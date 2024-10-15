#pragma once

#include <rclcpp/rclcpp.hpp>
#include <boost/circular_buffer.hpp>
#include <Eigen/Core>

#include "nomadz_kinematics/robot_model.hpp"
#include "nomadz_proprioception_msgs/msg/robot_model.hpp"
#include "nomadz_proprioception/arm_contact_model.hpp"

namespace nomadz_proprioception {

  class ArmContactEstimator {
  public:
    /**
     * udpates the arm contact model based on the current joint data and the previous joint requests, this is called in
     *the proprioception node.

     *@param measured_robot_model_msg The robot model accodrding to the current joint data
     *@param requested_robot_model_msg The robot model according to the previous joint requests

     */
    void update(const nomadz_kinematics::RobotModel& measured_robot_model,
                const nomadz_proprioception_msgs::msg::RobotModel& requested_robot_model_msg);
    /**
     * get function which returns the armContactModel *
     * @return the armContactModel
     */
    const ArmContactModel& getArmContactModel() const { return arm_contact_model_; }
    /**
     * resets the arm contact model to default values and clears the error buffers. overwritting the armcontactmodel with a
     * new one.
     */
    void reset();

  private:
    static constexpr int FRAME_BUFFER_SIZE = 5;
    static constexpr int ERROR_BUFFER_SIZE = 30;

    boost::circular_buffer<Eigen::Vector3f> hand_positions_buffer_{FRAME_BUFFER_SIZE};
    boost::circular_buffer<Eigen::Vector2f> left_error_buffer_{ERROR_BUFFER_SIZE};
    boost::circular_buffer<Eigen::Vector2f> right_error_buffer_{ERROR_BUFFER_SIZE};

    float error_x_threshold_ = 0.04F;   // radians = 4.0°;   // Maximum divergence of arm angleX (in degrees) that is not
                                        // treated as an obstacle detection
    float error_y_threshold_ = 0.0524F; // radians = 3.0°;  // Maximum divergence of arm angleY (in degrees) that is not
                                        // treated as an obstacle detection */
    Eigen::Vector2f left_;
    Eigen::Vector2f right_;
    rclcpp::Time start_time_left_;
    rclcpp::Time start_time_right_;
    ArmContactModel arm_contact_model_;

    /** Calculates the error between the current hand positions and previous requested hand positions and fills it into
     * the buffer.
     *
     * @param is_right_side The side of the arm to check. (if true, the right arm is checked, otherwise the left arm is
     * checked)
     * @param current_hand_positions the current pose of the hand
     * @param requested_hand_positions the requested pose of the hand based on the previous motor commands
     */
    void
    calculateError(bool is_right_side, Eigen::Affine3f& current_hand_positions, Eigen::Affine3f& requested_hand_positions);

    // // NOTE(franzbu): a correction factor might be useful for the future to adapt the scaling of the error readings,
    // previous implementation included odometry changes. float calculateCorrectionFactor(const Eigen::Affine3f fore_arm,
    // Eigen::Vector2f& last_arm_pos);

    /** Dertemines the push direction for an arm. That is, the direction in which the specified arm is being
     * pushed. Therefore, the object resulting in estimated push direction / interaction force is positioned opposite of
     * the pushdirection. The directions are given as Forward, Backward, Left, Right or None from the robots perspective.
     *
     * @param contactx  is a bool value, true if the error.x is above the x-axis threshold?
     * @param contacty is a bool value, true if the error.y is above the x-axis threshold?
     * @param error is the error vector.
     * @return The direction in which the specified arm is being pushed.
     */
    static PushDirection getDirection(bool contact_x, bool contact_y, Eigen::Vector2f error);
  };
} // namespace nomadz_proprioception

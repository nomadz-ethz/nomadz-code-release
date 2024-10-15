#include "nomadz_modeling/world_model_node.hpp"

#include <algorithm>
#include <vector>

#include "nomadz_core/geometry/twist.hpp"
#include "nomadz_core/geometry/ros_conversion.hpp"
#include "nomadz_modeling/pose_registrator.hpp"
#include "nomadz_modeling/ros_conversion.hpp"
#include "nomadz_modeling/self_locator.hpp"
#include "nomadz_motion_control_msgs/special_action_request_enums.hpp"
#include "nomadz_communication_msgs/robocup_game_control_data_enums.hpp"

namespace nomadz_modeling {

  WorldModelNode::WorldModelNode(const rclcpp::NodeOptions& options)
      : Node("world_model", options),
        parameters_(self_world_model_params::ParamListener(get_node_parameters_interface()).get_params()),
        self_locator_(self_locator_parameters::ParamListener(get_node_parameters_interface()).get_params()),
        ball_locator_(10), last_sensor_update_time_(this->now()),
        last_ball_seen_time_(this->now() - rclcpp::Duration::from_seconds(BALL_NOT_SEEN_TIMEOUT_S)) {

    world_model_pub_ = this->create_publisher<WorldModelMsgT>("modeling/world_model", 5);
    debug_ukf_state_pub_ = this->create_publisher<RobotPoseListMsgT>("ukf_debug_states", 1);
    registered_lines_pub_ = this->create_publisher<RegisteredLinesMsgT>("registered_lines", 1);

    motion_request_sub_ = this->create_subscription<MotionRequestMsgT>(
      "behavior/motion_request", 1, [this](MotionRequestMsgT::ConstSharedPtr msg) { setMotionSafety(msg); });

    motion_info_sub_ = this->create_subscription<MotionInfoMsgT>(
      "motion_control/motion_info", 1, [this](MotionInfoMsgT::ConstSharedPtr msg) { locatorMotionUpdate(msg); });

    field_perception_sub_ = this->create_subscription<FieldPerceptionDataMsgT>(
      "field_perception_data", 1, [this](FieldPerceptionDataMsgT::ConstSharedPtr msg) {
        if (is_motion_safe_) {
          locatorSensorUpdate(msg);
        }
      });

    ball_sub_ = this->create_subscription<BallMsgT>("ball", 1, [this](BallMsgT::ConstSharedPtr msg) { ballUpdate(msg); });

    ego_status_sub_ = this->create_subscription<EgoStatusMsgT>(
      "behavior/ego_status", 1, [this](EgoStatusMsgT::ConstSharedPtr msg) { checkPenalizedOrFallen(msg); });

    game_status_sub_ = this->create_subscription<GameStatusMsgT>(
      "behavior/game_status", 1, [this](GameStatusMsgT::ConstSharedPtr msg) { checkWalkInGameState(msg); });

    timer_ = this->create_wall_timer(std::chrono::milliseconds(30), [this] { worldModelCallback(); });
  }

  void WorldModelNode::setMotionSafety(const MotionRequestMsgT::ConstSharedPtr msg) {
    const auto special_action_type = static_cast<nomadz_motion_control_msgs::SpecialActionType>(msg->motion_type);
    is_motion_safe_ =
      (msg->motion_type == 0 && (special_action_type == nomadz_motion_control_msgs::SpecialActionType::STAND ||
                                 special_action_type == nomadz_motion_control_msgs::SpecialActionType::STAND_HIGH)) ||
      msg->motion_type == 1; // Note(emilio): (SpecialAction and (Stand or StandHigh)) or Walk
  }

  void WorldModelNode::locatorMotionUpdate(const MotionInfoMsgT::ConstSharedPtr msg) {
    const bool is_moving =
      (std::pow(msg->odometry_offset.linear.x, 2) + std::pow(msg->odometry_offset.linear.y, 2)) > 0.0001 * 0.0001;
    is_moving_.push_back(is_moving);
    self_locator_.motionUpdate(nomadz_core::toVector3(nomadz_core::unpackTwist2D(msg->odometry_offset)));
  }

  void WorldModelNode::locatorSensorUpdate(const FieldPerceptionDataMsgT::ConstSharedPtr msg) {

    std::vector<PerceivedFeaturePose> no_poses;
    bool register_something = self_locator_.sensorUpdate(
      unpackLandmarks(msg->landmarks), unpackIntersections(msg->intersections), no_poses, unpackLines(msg->field_lines));

    if (register_something) {
      last_sensor_update_time_ = this->now();
    }

    if (parameters_.publish_ukf_debug) {
      RegisteredLinesMsgT registered_lines_msg;
      registered_lines_msg.header.stamp = this->now();
      for (const auto& line : self_locator_.debug_registered_lines) {
        nomadz_vision_msgs::msg::FieldLine field_line_percept;
        nomadz_vision_msgs::msg::FieldLine field_line_gt;

        field_line_percept.start.x = line.percept.from.x();
        field_line_percept.start.y = line.percept.from.y();
        field_line_gt.start.x = line.model.from.x();
        field_line_gt.start.y = line.model.from.y();
        field_line_percept.end.x = line.percept.to.x();
        field_line_percept.end.y = line.percept.to.y();
        field_line_gt.end.x = line.model.to.x();
        field_line_gt.end.y = line.model.to.y();
        registered_lines_msg.seen_lines.push_back(field_line_percept);
        registered_lines_msg.registered_lines.push_back(field_line_gt);
      }

      for (const auto& landmark : self_locator_.debug_registered_landmarks) {
        geometry_msgs::msg::Point seen_landmark;
        geometry_msgs::msg::Point registered_landmark;

        seen_landmark.x = landmark.percept.x();
        seen_landmark.y = landmark.percept.y();
        registered_landmark.x = landmark.model.x();
        registered_landmark.y = landmark.model.y();
        registered_lines_msg.seen_landmarks.push_back(seen_landmark);
        registered_lines_msg.registered_landmarks.push_back(registered_landmark);
      }
      registered_lines_pub_->publish(registered_lines_msg);
    }
  }

  void WorldModelNode::ballUpdate(const BallMsgT::ConstSharedPtr msg) {
    Eigen::Vector2f relative_ball_position = Eigen::Vector2f{msg->position.x, msg->position.y};
    if (msg->ball_seen) {
      last_ball_seen_time_ = this->now();
      ball_locator_.update(relative_ball_position, msg->ball_seen);
    }
  }

  void WorldModelNode::checkPenalizedOrFallen(const EgoStatusMsgT::ConstSharedPtr msg) {
    if (msg->is_penalized) { // TODO(emilio): stricter conditions needed in general
      self_locator_.initParticlesFromPenalty();
    } else if (msg->has_fallen) { // TODO(emilio): stricter conditions needed in general
      self_locator_.initParticlesIfFalling();
    }
  }

  void WorldModelNode::checkWalkInGameState(const GameStatusMsgT::ConstSharedPtr msg) {
    // NOTE(@naefjo): init the particle filter while we're not in the playing state
    const auto current_state = static_cast<nomadz_communication_msgs::GameState>(msg->game_state);
    if (last_game_state_ == nomadz_communication_msgs::GameState::STANDBY &&
        current_state == nomadz_communication_msgs::GameState::READY) {
      // NOTE(@naefjo): If we switch to the ready state we initialize the sensor update timeout
      // since the we can't be not localized previously
      last_sensor_update_time_ = this->now();
      self_locator_.initParticles();
    }
    last_game_state_ = current_state;
  }

  void WorldModelNode::worldModelCallback() {

    // NOTE(@naefjo): If we are lost we try to recover by resetting the orientation
    // of the particles uniformly. The rationale is that the pose estimate should not have
    // drifted too much if it was okay before.
    if (robotPoseIsLost()) {
      self_locator_.initParticlesIfLost();
    }
    WorldModelMsgT world_model;
    world_model.header.stamp = this->now();
    world_model.robot_pose = createRobotPoseMsg(self_locator_.getBestPose(), robotPoseIsValid(), robotPoseIsLost());
    if ((this->now() - last_ball_seen_time_).seconds() < BALL_NOT_SEEN_TIMEOUT_S) {
      world_model.ball_model.position.x = ball_locator_.xFiltered();
      world_model.ball_model.position.y = ball_locator_.yFiltered();
      world_model.ball_model.valid = true;
      world_model.ball_model.lost = false;
    } else {
      ball_locator_.reset();
      world_model.ball_model.valid = false;
      world_model.ball_model.lost = true;
    }
    world_model_pub_->publish(world_model);

    if (parameters_.publish_ukf_debug) {
      RobotPoseListMsgT particle_poses;
      std::vector<nomadz_core::Pose2D> particle_pose2d = self_locator_.getParticlePoses();
      for (const nomadz_core::Pose2D& particle_pose2d : particle_pose2d) {
        particle_poses.robot_poses.push_back(createPose2DMsgFromPose(particle_pose2d));
      }
      particle_poses.header.stamp = this->now();
      debug_ukf_state_pub_->publish(particle_poses);
    }
  }

  bool WorldModelNode::robotPoseIsLost() const {
    bool sensor_msgs_timed_out = getTimeSince(last_sensor_update_time_) > parameters_.lost_timeout;
    bool is_in_relevant_game_state = last_game_state_ == nomadz_communication_msgs::GameState::READY ||
                                     last_game_state_ == nomadz_communication_msgs::GameState::SET ||
                                     last_game_state_ == nomadz_communication_msgs::GameState::PLAYING;

    const bool was_moving = std::any_of(is_moving_.begin(), is_moving_.end(), [](bool is_moving) { return is_moving; });
    return was_moving && sensor_msgs_timed_out && is_in_relevant_game_state;
  }

} // namespace nomadz_modeling

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nomadz_modeling::WorldModelNode)

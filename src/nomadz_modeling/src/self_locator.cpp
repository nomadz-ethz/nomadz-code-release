#include "nomadz_modeling/self_locator.hpp"

#include <chrono>
#include <memory>
#include <algorithm>

#include "nomadz_core/math/constants.hpp"
#include "nomadz_core/math/distributions.hpp"
#include "nomadz_modeling/landmark_registrator.hpp"
#include "nomadz_modeling/particle.hpp"
#include "nomadz_modeling/self_locator_utils.hpp"
#include "nomadz_configuration/field_dimensions.hpp"

namespace nomadz_modeling {

  SelfLocator::SelfLocator(const self_locator_parameters::Params parameters) : parameters_(parameters) {
    initParticles();
  }

  void SelfLocator::initParticles() {
    nomadz_configuration::FieldDimensions field_dimensions;

    Eigen::Vector3f init_state =
      computeInitialState(settings_.player_id, field_dimensions.field_length / 2.F, field_dimensions.field_width / 2.F);
    Eigen::Matrix3f init_cov = Eigen::Matrix3f::Zero();
    for (int i = 0; i < 3; i++) {
      init_cov(i, i) += static_cast<float>(std::pow(INIT_DEVIATION_(i), 2));
    }

    particles_.clear();
    for (int i = 0; i < parameters_.number_of_samples; ++i) {
      Particle particle{init_state, init_cov};
      particles_.push_back(particle);
    }
    updateRobotPose();
  }

  void SelfLocator::initParticlesFromPenalty() {
    nomadz_configuration::Dims dims;

    Eigen::Matrix3f init_cov = Eigen::Matrix3f::Zero();
    for (int i = 0; i < 3; i++) {
      init_cov(i, i) += static_cast<float>(std::pow(INIT_PENALTY_DEVIATION_(i), 2));
    }

    particles_.clear();
    const float y_left = (dims.y_pos_left_sideline + dims.y_pos_left_field_border) / 2.F;
    const float y_right = (dims.y_pos_right_sideline + dims.y_pos_right_field_border) / 2.F;
    for (int i = 0; i < parameters_.number_of_samples / 2; ++i) {
      Eigen::Vector3f init_state{dims.x_pos_own_penalty_mark, y_left, -nomadz_core::constants::PI_2};
      Particle particle{init_state, init_cov};
      particles_.push_back(particle);
    }
    for (int i = static_cast<int>(parameters_.number_of_samples) / 2; i < parameters_.number_of_samples; ++i) {
      Eigen::Vector3f init_state{dims.x_pos_own_penalty_mark, y_right, nomadz_core::constants::PI_2};
      Particle particle{init_state, init_cov};
      particles_.push_back(particle);
    }
    updateRobotPose();
  }

  void SelfLocator::initParticlesIfLost() {
    Eigen::Matrix3f init_cov = Eigen::Matrix3f::Zero();
    for (int i = 0; i < 3; i++) {
      init_cov(i, i) += static_cast<float>(std::pow(INIT_LOST_DEVIATION_(i), 2));
    }

    particles_.clear();
    const float dtheta = nomadz_core::constants::PI_2;
    for (int i = 0; i < parameters_.number_of_samples; ++i) {
      Eigen::Vector3f init_state{
        robot_pose_.x, robot_pose_.y, nomadz_core::angle::normalize(static_cast<float>(i) * dtheta)};
      Particle particle{init_state, init_cov};
      particles_.push_back(particle);
    }
    updateRobotPose();
  }

  void SelfLocator::initParticlesIfFalling() {
    Eigen::Matrix3f init_cov = Eigen::Matrix3f::Zero();
    for (int i = 0; i < 3; i++) {
      init_cov(i, i) += static_cast<float>(std::pow(INIT_FALL_DEVIATION_(i), 2));
    }

    particles_.clear();
    const float dtheta = nomadz_core::constants::PI;
    for (int i = 0; i < parameters_.number_of_samples; ++i) {
      Eigen::Vector3f init_state{
        robot_pose_.x, robot_pose_.y, nomadz_core::angle::normalize(static_cast<float>(i) * dtheta)};
      Particle particle{init_state, init_cov};
      particles_.push_back(particle);
    }
    updateRobotPose();
  }

  void SelfLocator::motionUpdate(const Eigen::Vector3f& odometry_offset) {

    const float dist = odometry_offset.head<2>().norm();
    const float angle = std::abs(odometry_offset.z());

    const auto x_error = static_cast<float>(std::max(std::abs(odometry_offset.x() * parameters_.major_dir_trans_weight),
                                                     std::abs(odometry_offset.y() * parameters_.minor_dir_trans_weight)));
    const auto y_error = static_cast<float>(std::max(std::abs(odometry_offset.y() * parameters_.major_dir_trans_weight),
                                                     std::abs(odometry_offset.x() * parameters_.minor_dir_trans_weight)));
    const auto theta_error = static_cast<float>(
      std::max(dist * parameters_.moved_dist_weight_rotation_noise, angle * parameters_.angle_weight_noise));

    for (Particle& particle : particles_) {
      float x_offset = (odometry_offset.x() - x_error) + 2.F * x_error * nomadz_core::random::uniform();
      float y_offset = (odometry_offset.y() - y_error) + 2.F * y_error * nomadz_core::random::uniform();
      float theta_offset = odometry_offset.z() + nomadz_core::random::uniform(-theta_error, theta_error);
      particle.predict(Eigen::Vector3f{x_offset, y_offset, theta_offset});
    }

    updateRobotPose();
  }

  bool SelfLocator::sensorUpdate(const std::vector<PerceivedLandmark>& perceived_landmarks,
                                 const std::vector<PerceivedIntersection>& perceived_intersections,
                                 const std::vector<PerceivedFeaturePose>& perceived_feature_poses,
                                 const std::vector<PerceivedLine>& perceived_lines) {
    // TODO(emilio): check motion safety before updating
    bool uses_landmarks = (!perceived_landmarks.empty()) || (!perceived_intersections.empty());
    bool uses_robot_poses = !perceived_feature_poses.empty();
    bool uses_lines = !perceived_lines.empty();
    bool validities_have_been_updated = false;

    debug_registered_lines.clear();
    debug_registered_landmarks.clear();
    bool register_something = false;
    for (Particle& particle : particles_) {
      std::vector<RegisteredLandmark> landmarks;
      std::vector<RegisteredRobotPose> robot_poses;
      std::vector<RegisteredLine> lines;

      Eigen::Vector2f num_den = Eigen::Vector2f::Zero();
      if (uses_landmarks) {
        num_den += sensorUpdateLandmark(particle, perceived_landmarks, perceived_intersections, landmarks);
        if (!landmarks.empty()) {
          register_something = true;
        }
      }
      if (uses_robot_poses) {
        num_den += sensorUpdateFeaturePose(particle, perceived_feature_poses, robot_poses);
        if (!robot_poses.empty()) {
          register_something = true;
        }
      }
      if (uses_lines) {
        num_den += sensorUpdateLine(particle, perceived_lines, lines);
        if (!lines.empty()) {
          register_something = true;
        }
      }

      // NOTE(@naefjo): This means we didnt register anything (i think) so we skip the rest of the update.
      if (num_den[1] < 1) {
        continue;
      }

      const float new_validity = num_den[0] / num_den[1];
      const float past_validity = particle.getValidity();
      particle.updateValidity(static_cast<int>(parameters_.number_of_considered_frames_for_validity), new_validity);
      if (past_validity != particle.getValidity()) {
        validities_have_been_updated = true;
      }
    }
    updateRobotPose();

    if (validities_have_been_updated) {
      resampling();
    }

    for (auto& particle : particles_) {
      if (!isOnCarpet(particle.getPose())) {
        particle.invalidate();
      }
    }
    return register_something;
  }

  Eigen::Vector2f SelfLocator::sensorUpdateLine(Particle& particle,
                                                const std::vector<PerceivedLine>& perceived_lines,
                                                std::vector<RegisteredLine>& lines) {
    line_registrator_.registerLines(particle.getPose(), perceived_lines, lines);
    for (const RegisteredLine& line : lines) {
      particle.updateByLine(line);
    }
    debug_registered_lines.insert(debug_registered_lines.end(), lines.begin(), lines.end());
    auto numerator = static_cast<float>(parameters_.validity_factor_line_measurement) * static_cast<float>(lines.size()) /
                     static_cast<float>(perceived_lines.size());
    auto denominator = static_cast<float>(parameters_.validity_factor_line_measurement);
    return Eigen::Vector2f{numerator, denominator};
  }

  Eigen::Vector2f SelfLocator::sensorUpdateLandmark(Particle& particle,
                                                    const std::vector<PerceivedLandmark>& perceived_landmarks,
                                                    const std::vector<PerceivedIntersection>& perceived_intersections,
                                                    std::vector<RegisteredLandmark>& landmarks) {
    landmark_registrator_.registerLandmarks(particle.getPose(), perceived_landmarks, landmarks);
    landmark_registrator_.registerLandmarks(particle.getPose(), perceived_intersections, landmarks);
    for (const RegisteredLandmark& landmark : landmarks) {
      particle.updateByLandmark(landmark);
    }
    debug_registered_landmarks.insert(debug_registered_landmarks.end(), landmarks.begin(), landmarks.end());
    auto numerator = static_cast<float>(parameters_.validity_factor_landmark_measurement) *
                     static_cast<float>(landmarks.size()) /
                     static_cast<float>(perceived_landmarks.size() + perceived_intersections.size());
    auto denominator = static_cast<float>(parameters_.validity_factor_landmark_measurement);
    return Eigen::Vector2f{numerator, denominator};
  }

  Eigen::Vector2f SelfLocator::sensorUpdateFeaturePose(Particle& particle,
                                                       const std::vector<PerceivedFeaturePose>& perceived_feature_poses,
                                                       std::vector<RegisteredRobotPose>& robot_poses) {
    pose_registrator_.registerPoses(particle.getPose(), perceived_feature_poses, robot_poses);
    for (const RegisteredRobotPose& robot_pose : robot_poses) {
      particle.updateByRobotPose(robot_pose);
    }
    auto numerator = static_cast<float>(parameters_.validity_factor_pose_measurement) *
                     static_cast<float>(robot_poses.size()) / static_cast<float>(perceived_feature_poses.size());
    auto denominator = static_cast<float>(parameters_.validity_factor_pose_measurement);
    return Eigen::Vector2f{numerator, denominator};
  }

  void SelfLocator::updateRobotPose() {
    float max_validity = -1.F;
    float min_variance = 0.F; // Initial value does not matter
    Particle current_best_particle = particles_[0];
    for (const Particle& particle : particles_) {
      const float val = particle.getValidity();
      if (val > max_validity) {
        max_validity = val;
        min_variance = particle.getCombinedVariance();
        current_best_particle = particle;
      } else if (val == max_validity) {
        const float variance = particle.getCombinedVariance();
        if (variance < min_variance) {
          max_validity = val;
          min_variance = variance;
          current_best_particle = particle;
        }
      }
    }
    if (last_best_particle_ != nullptr &&
        current_best_particle.getValidity() <= last_best_particle_->getValidity() * 0.75F) {
      robot_pose_ = last_best_particle_->getPose();
      float current_last_particle_validity = last_best_particle_->getValidity();
      last_best_particle_->updateValidity(1, current_last_particle_validity * 0.9F);
    } else {
      last_best_particle_ = std::make_unique<Particle>(current_best_particle);
      robot_pose_ = current_best_particle.getPose();
    }
  }

  void SelfLocator::resampling() {
    float average_weight = 0.F;
    for (Particle& particle : particles_) {
      particle.updateWeight(static_cast<float>(parameters_.base_validity_weight));
      average_weight += particle.getWeight();
    }
    average_weight /= static_cast<float>(particles_.size());

    std::vector<Particle> old_particles = std::vector<Particle>();
    old_particles.assign(particles_.begin(), particles_.end());

    int number_of_particles = static_cast<int>(old_particles.size());
    float next_pos = nomadz_core::random::uniform() * average_weight;
    float current_sum = 0.F;
    int j = 0;
    for (Particle& old_particle : old_particles) {
      current_sum += old_particle.getWeight();
      while (next_pos < current_sum && j < number_of_particles) {
        particles_[j] = old_particle;
        j++;
        next_pos += average_weight;
      }
    }

    // fill up missing samples (could happen in rare cases due to numerical imprecision / rounding / whatever) with new
    // poses:
    Eigen::Matrix3f default_cov = Eigen::Matrix3f::Zero();
    for (int i = 0; i < 3; ++i) {
      default_cov(i, i) = static_cast<float>(std::pow(DEFAULT_DEVIATION_(i), 2));
    }
    nomadz_configuration::Dims field_dimensions;
    for (; j < number_of_particles; ++j) {
      // TODO(emilio): Bhuman uses theAlternativeRobotPoseHypothesis. Understand what's the difference
      float x =
        std::clamp(robot_pose_.x, field_dimensions.x_pos_own_field_border, field_dimensions.x_pos_opponent_field_border);
      float y =
        std::clamp(robot_pose_.y, field_dimensions.y_pos_right_field_border, field_dimensions.y_pos_left_field_border);
      float theta = 2 * nomadz_core::constants::PI * nomadz_core::random::uniform();
      Eigen::Vector3f new_pose{x, y, theta};
      particles_[j] = Particle(new_pose, default_cov);
      particles_[j].updateValidity(1, average_weight);
    }
  }

  bool SelfLocator::isOnCarpet(const nomadz_core::Pose2D& robot_pose) {
    nomadz_configuration::Dims field_dimensions;
    return robot_pose.x <= field_dimensions.x_pos_opponent_field_border &&
           robot_pose.x >= field_dimensions.x_pos_own_field_border &&
           robot_pose.y <= field_dimensions.y_pos_left_field_border &&
           robot_pose.y >= field_dimensions.y_pos_right_field_border;
  }
} // namespace nomadz_modeling

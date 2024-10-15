#include "nomadz_modeling/particle.hpp"

#include "nomadz_core/geometry/angle.hpp"

namespace nomadz_modeling {

  Particle::Particle(const Eigen::Vector3f& init_state, const Eigen::Matrix3f& init_cov) {
    ukf_.init(init_state, init_cov);
  }

  void Particle::predict(const Eigen::Vector3f& odometry_offset) {
    auto dynamics = [&odometry_offset](Eigen::Vector3f& current_state) { dynamicsModel(current_state, odometry_offset); };
    Eigen::Matrix3f noise = Eigen::Matrix3f::Zero();
    for (int i = 0; i < 3; i++) {
      noise(i, i) += std::pow(FILTER_PROCESS_DEVIATION_(i), 2);
    }

    Eigen::Rotation2Df rotation{ukf_.mean.z()};
    Eigen::Vector2f rotate_odometry = rotation * odometry_offset.head<2>();
    noise(0, 0) += std::pow(rotate_odometry.x() * ODOMETRY_DEVIATION_.x(), 2);
    noise(1, 1) += std::pow(rotate_odometry.y() * ODOMETRY_DEVIATION_.y(), 2);
    noise(2, 2) += std::pow(odometry_offset.z() * ODOMETRY_DEVIATION_.z(), 2);
    noise(2, 2) += std::pow(rotate_odometry.x() * ODOMETRY_ROTATION_DEVIATION_.x(), 2);
    noise(2, 2) += std::pow(rotate_odometry.y() * ODOMETRY_DEVIATION_.y(), 2);

    ukf_.predict(dynamics, noise);
    ukf_.mean.z() = nomadz_core::angle::normalize(ukf_.mean.z());
  }

  void Particle::updateByLandmark(const RegisteredLandmark& landmark) {
    auto measurement_model = [model = landmark.model](const Eigen::Vector3f& state) {
      return landmarkMeasurementModel(state, model);
    };
    ukf_.update<2>(landmark.percept, measurement_model, landmark.cov_percept);
  }

  void Particle::updateByRobotPose(const RegisteredRobotPose& robot_pose) {
    auto measurement_model = robotPoseMeasurementModel;
    ukf_.update<3>(nomadz_core::toVector3(robot_pose.percept), measurement_model, robot_pose.cov_percept);
  }

  void Particle::updateByLine(const RegisteredLine& line) {

    const nomadz_core::Pose2D pose = getPose();
    const float delta_angle_alternative = nomadz_core::angle::normalize(line.measured_angle_alternative - pose.theta);
    const float delta_angle = nomadz_core::angle::normalize(line.measured_angle - pose.theta);
    const float measured_angle =
      std::abs(delta_angle_alternative) < std::abs(delta_angle) ? line.measured_angle_alternative : line.measured_angle;

    const Eigen::Rotation2Df rotation{measured_angle};

    const Eigen::Vector2f orthogonal_projection = rotation * line.orthogonal_projection;
    Eigen::Matrix2f cov = rotation * line.cov_percept_center * rotation.inverse();

    auto measurement_model = [vertical = line.parallel_to_worl_model_x_axis](const Eigen::Vector3f& state) {
      return lineMeasurementModel(state, vertical);
    };

    int i = line.parallel_to_worl_model_x_axis ? 1 : 0;
    float measured_coord = line.model.from(i) - orthogonal_projection(i);
    float variance_coord = cov(i, i);
    const float angle_variance = static_cast<float>(
      std::pow(std::atan(std::sqrt(4.F * variance_coord / (line.percept.from - line.percept.to).squaredNorm())), 2));
    Eigen::Matrix2f measurement_noise;
    measurement_noise << variance_coord, 0.F, 0.F, angle_variance;
    ukf_.update<2>(Eigen::Vector2f{measured_coord, measured_angle}, measurement_model, measurement_noise);
  }
} // namespace nomadz_modeling

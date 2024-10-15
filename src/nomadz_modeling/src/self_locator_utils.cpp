#include "nomadz_modeling/self_locator_utils.hpp"

#include "nomadz_core/math/constants.hpp"

namespace nomadz_modeling {

  void dynamicsModel(Eigen::Vector3f& state, const Eigen::Vector3f& odometry_offset) {
    Eigen::Rotation2Df rotation{state.z()};
    Eigen::Vector2f rotate_odometry = rotation * odometry_offset.head<2>();
    state += Eigen::Vector3f{rotate_odometry.x(), rotate_odometry.y(), odometry_offset.z()};
  }

  Eigen::Vector2f landmarkMeasurementModel(const Eigen::Vector3f& state, const Eigen::Vector2f& model) {
    Eigen::Affine2f transform = Eigen::Affine2f::Identity();
    transform.rotate(state.z());
    transform.translation() << state.x(), state.y();
    return transform.inverse() * model;
  }

  Eigen::Vector3f robotPoseMeasurementModel(const Eigen::Vector3f& state) {
    return state;
  }

  Eigen::Vector2f lineMeasurementModel(const Eigen::Vector3f& state, bool vertical) {
    if (vertical) {
      return Eigen::Vector2f{state.y(), state.z()};
    }
    return Eigen::Vector2f{state.x(), state.z()};
  }

  Eigen::Vector3f computeInitialState(const int player_number, const float half_field_length, const float half_field_width) {
    float x;
    float y;
    float theta;

    if (player_number % 2 == 0) {
      float delta_x = half_field_length / 4.F;
      x = static_cast<float>(player_number) / 2.F * delta_x - half_field_length;
      y = half_field_width;
      theta = -nomadz_core::constants::PI_2;
    } else {
      float delta_x = half_field_length / 5.F;
      x = (static_cast<float>(player_number) + 1.F) / 2.F * delta_x - half_field_length;
      y = -half_field_width;
      theta = nomadz_core::constants::PI_2;
    }
    return Eigen::Vector3f{x, y, theta};
  }
} // namespace nomadz_modeling

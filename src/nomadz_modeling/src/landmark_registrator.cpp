#include "nomadz_modeling/landmark_registrator.hpp"

#include <cmath>
#include <limits>

#include "nomadz_core/geometry/angle.hpp"

namespace nomadz_modeling {

  void LandmarkRegistrator::registerLandmarks(const nomadz_core::Pose2D& robot_pose,
                                              const std::vector<PerceivedLandmark>& perceived_landmarks,
                                              std::vector<RegisteredLandmark>& registered_landmarks) const {
    for (const PerceivedLandmark& perceived_landmark : perceived_landmarks) {
      auto field_landmark = getCorrespondingLandmarkInWorldModel(robot_pose, perceived_landmark);
      if (field_landmark.has_value()) {
        RegisteredLandmark registered_landmark;
        registered_landmark.percept = perceived_landmark.percept;
        registered_landmark.model = field_landmark.value();
        registered_landmark.cov_percept =
          perceived_landmark.cov_percept +
          ADDITIONAL_NOISE_; // Note(emilio): remove added noise once measurement has covariance
        registered_landmarks.push_back(registered_landmark);
      }
    }
  }

  void LandmarkRegistrator::registerLandmarks(const nomadz_core::Pose2D& robot_pose,
                                              const std::vector<PerceivedIntersection>& perceived_intersections,
                                              std::vector<RegisteredLandmark>& registered_landmarks) const {
    for (const PerceivedIntersection& perceived_intersection : perceived_intersections) {
      auto field_landmark = getCorrespondingIntersectionInWorldModel(robot_pose, perceived_intersection);
      if (field_landmark.has_value()) {
        RegisteredLandmark registered_landmark;
        registered_landmark.percept = perceived_intersection.percept;
        registered_landmark.model = field_landmark.value();
        registered_landmark.cov_percept =
          perceived_intersection.cov_percept +
          ADDITIONAL_NOISE_; // Note(emilio): remove added noise once measurement has covariance
        registered_landmarks.push_back(registered_landmark);
      }
    }
  }

  std::optional<Eigen::Vector2f>
  LandmarkRegistrator::getCorrespondingLandmarkInWorldModel(const nomadz_core::Pose2D& robot_pose,
                                                            const PerceivedLandmark& perceived_landmark) const {
    if (perceived_landmark.landmark_type == LandmarkType::PENALTY_MARK) {
      return getCorrespondingPenaltyMarkInWorldModel(robot_pose, perceived_landmark);
    }
    return getCorrespondingCircleInWorldModel(robot_pose, perceived_landmark);
  }

  std::optional<Eigen::Vector2f>
  LandmarkRegistrator::getCorrespondingPenaltyMarkInWorldModel(const nomadz_core::Pose2D& robot_pose,
                                                               const PerceivedLandmark& perceived_penalty_mark) const {
    const Eigen::Vector2f percept_penalty_mark_world = nomadz_core::toAffine2(robot_pose) * perceived_penalty_mark.percept;
    Eigen::Vector2f model_penalty_mark_world =
      percept_penalty_mark_world.x() <= 0.F ? OWN_PENALTY_MODEL_ : OPPONENT_PENALTY_MODEL_;
    if ((percept_penalty_mark_world - model_penalty_mark_world).norm() <= MAX_PENALTY_MARK_ERROR_) {
      return model_penalty_mark_world;
    }
    return {};
  }

  std::optional<Eigen::Vector2f>
  LandmarkRegistrator::getCorrespondingCircleInWorldModel(const nomadz_core::Pose2D& robot_pose,
                                                          const PerceivedLandmark& perceived_circle) const {
    const Eigen::Vector2f percept_circle_world = nomadz_core::toAffine2(robot_pose) * perceived_circle.percept;
    Eigen::Vector2f model_circle_world{0.F, 0.F}; // We assume that the CENTER circle is always at (0,0)
    if ((percept_circle_world - model_circle_world).norm() <= MAX_CENTER_CIRCLE_ERROR_) {
      return model_circle_world;
    }
    return {};
  }

  std::optional<Eigen::Vector2f>
  LandmarkRegistrator::getCorrespondingIntersectionInWorldModel(const nomadz_core::Pose2D& robot_pose,
                                                                const PerceivedIntersection& perceived_intersection) const {
    const Eigen::Vector2f percept_intersection_world = nomadz_core::toAffine2(robot_pose) * perceived_intersection.percept;
    const std::vector<Eigen::Vector2f> intersection_list = getCorrespondingIntersections(robot_pose, perceived_intersection);

    Eigen::Vector2f model_intersection_world;
    float min_distance = std::numeric_limits<float>::infinity();
    for (const Eigen::Vector2f& intersection : intersection_list) {
      const float distance = (percept_intersection_world - intersection).norm();
      if (distance < min_distance) {
        min_distance = distance;
        model_intersection_world = intersection;
      }
    }

    // Check, if closest intersection is close enough:
    if (min_distance < MAX_INTERSECTION_ERROR_) {
      return model_intersection_world;
    }
    return {};
  }

  std::vector<Eigen::Vector2f>
  LandmarkRegistrator::getCorrespondingIntersections(const nomadz_core::Pose2D& robot_pose,
                                                     const PerceivedIntersection& perceived_intersection) const {
    if (perceived_intersection.intersection_type == IntersectionType::X) {
      return corners_.X_CORNER;
    }
    int section = getIntersectionDirectionTo90DegreeSection(robot_pose, perceived_intersection);

    if (perceived_intersection.intersection_type == IntersectionType::T) {
      switch (section) {
      case 0:
        return corners_.T_CORNER_0;
      case 90:
        return corners_.T_CORNER_90;
      case 180:
        return corners_.T_CORNER_180;
      default:
        return corners_.T_CORNER_270;
      }
    }

    switch (section) {
    case 0:
      return corners_.L_CORNER_0;
    case 90:
      return corners_.L_CORNER_90;
    case 180:
      return corners_.L_CORNER_180;
    default:
      return corners_.L_CORNER_270;
    }
  }

  int LandmarkRegistrator::getIntersectionDirectionTo90DegreeSection(const nomadz_core::Pose2D& robot_pose,
                                                                     const PerceivedIntersection& perceived_intersection) {
    Eigen::Rotation2Df rotation{robot_pose.theta};
    Eigen::Vector2f direction_in_field_coordinates = rotation * perceived_intersection.dir;
    const float rad = std::atan2(direction_in_field_coordinates.y(), direction_in_field_coordinates.x());
    const float degrees = nomadz_core::angle::toDegrees(rad) + 180.F;
    if (degrees < 45.F || degrees >= 315.F) {
      return 180;
    }
    if (degrees >= 45.F && degrees < 135.F) {
      return 270;
    }
    if (degrees >= 135.F && degrees < 225.F) {
      return 0;
    }
    // equivalent to (degrees >= 225 && degrees < 315)
    return 90;
  }

} // namespace nomadz_modeling

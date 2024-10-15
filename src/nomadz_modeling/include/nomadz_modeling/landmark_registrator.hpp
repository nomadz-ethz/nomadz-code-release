#pragma once

#include <optional>
#include <vector>

#include <Eigen/Core>

#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_configuration/field_dimensions.hpp"

namespace nomadz_modeling {

  /**
   * Enumeration for the types of landmarks.
   * PENALTY_MARK: Represents a penalty mark.
   * CIRCLE: Represents a center circle.
   */
  enum class LandmarkType { PENALTY_MARK = 0, CIRCLE };

  /**
   * Enumeration for the types of intersections (L, T, X).
   * They are inclusive, so:
   * - X contains T and L
   * - T contains L
   */
  enum class IntersectionType { L = 0, T, X };

  struct PerceivedLandmark {
    // The position of the perceived landmark in the robot frame
    Eigen::Vector2f percept;
    // The covariance of the landmark measurement
    Eigen::Matrix2f cov_percept;
    // The type of the landmark
    LandmarkType landmark_type;
  };

  struct PerceivedIntersection {
    // The position of the perceived intersection in the robot frame
    Eigen::Vector2f percept;
    // The covariance of the intersection measurement
    Eigen::Matrix2f cov_percept;
    // The primary direction of the intersection line.
    Eigen::Vector2f dir;

    IntersectionType intersection_type;
  };

  struct RegisteredLandmark {
    // The position of the perceived landmark in the robot frame
    Eigen::Vector2f percept;
    // The position of the original landmark in the world frame
    Eigen::Vector2f model;
    // The covariance of the landmark measurement
    Eigen::Matrix2f cov_percept;
  };

  class LandmarkRegistrator {

  public:
    LandmarkRegistrator() = default;

    /**
     * Register the perceived landmarks based on the assumed robot pose
     * @param robot_pose The assumed robot pose
     * @param perceived_landmarks A list containing all the perceived landmarks to analyze
     * @param registered_landmarks A list to which all compatible measurements are added
     */
    void registerLandmarks(const nomadz_core::Pose2D& robot_pose,
                           const std::vector<PerceivedLandmark>& perceived_landmarks,
                           std::vector<RegisteredLandmark>& registered_landmarks) const;

    /**
     * Register the perceived intersections based on the assumed robot pose
     * @param robot_pose The assumed robot pose
     * @param perceived_landmarks A list containing all the perceived intersections to analyze
     * @param registered_landmarks A list to which all compatible measurements are added
     */
    void registerLandmarks(const nomadz_core::Pose2D& robot_pose,
                           const std::vector<PerceivedIntersection>& perceived_intersections,
                           std::vector<RegisteredLandmark>& registered_landmarks) const;

  private:
    /**
     * Determine if the perceived landmark matches one of the landmarks on the field.
     * @param robot_pose The assumed pose of the robot.
     * @param perceived_landmark The perceived landmark in the robot frame.
     * @return An optional containing the corresponding landmark in the world model.
     */
    std::optional<Eigen::Vector2f> getCorrespondingLandmarkInWorldModel(const nomadz_core::Pose2D& robot_pose,
                                                                        const PerceivedLandmark& perceived_landmark) const;

    std::optional<Eigen::Vector2f>
    getCorrespondingPenaltyMarkInWorldModel(const nomadz_core::Pose2D& robot_pose,
                                            const PerceivedLandmark& perceived_penalty_mark) const;

    std::optional<Eigen::Vector2f> getCorrespondingCircleInWorldModel(const nomadz_core::Pose2D& robot_pose,
                                                                      const PerceivedLandmark& perceived_circle) const;

    std::optional<Eigen::Vector2f>
    getCorrespondingIntersectionInWorldModel(const nomadz_core::Pose2D& robot_pose,
                                             const PerceivedIntersection& perceived_intersection) const;

    /**
     * Function to get all potential corresponding intersections in the world model given the robot's pose.
     * @param robot_pose The assumed pose of the robot.
     * @return A vector of potential corresponding intersection positions in the world model.
     */
    std::vector<Eigen::Vector2f> getCorrespondingIntersections(const nomadz_core::Pose2D& robot_pose,
                                                               const PerceivedIntersection& perceived_intersection) const;

    /**
     * Function to get the direction of the intersection relative to the 90-degree section.
     * @param robot_pose The assumed pose of the robot.
     * @return An integer representing the direction to the 90-degree section of the intersection.
     */
    static int getIntersectionDirectionTo90DegreeSection(const nomadz_core::Pose2D& robot_pose,
                                                         const PerceivedIntersection& perceived_intersection);

    nomadz_configuration::FieldLines world_model_field_lines_;
    nomadz_configuration::Corners corners_;

    // Maximum allowed error for penalty mark registration
    const float MAX_PENALTY_MARK_ERROR_ = 1.5F;
    // Position of the own penalty mark in the world model
    const Eigen::Vector2f OWN_PENALTY_MODEL_{world_model_field_lines_.x_pos_own_penalty_mark, 0.F};
    // Position of the opponent's penalty mark in the world model
    const Eigen::Vector2f OPPONENT_PENALTY_MODEL_{world_model_field_lines_.x_pos_opponent_penalty_mark, 0.F};
    // Maximum allowed error for center circle registration
    const float MAX_CENTER_CIRCLE_ERROR_ = 1.9F;
    // Maximum allowed error for intersection registration in meters
    const float MAX_INTERSECTION_ERROR_ = 0.5F;
    // Cov noise added after registration Note(emilio): set to 0 once the measurements have accurate cov
    const Eigen::Matrix2f ADDITIONAL_NOISE_ = 0.05F * Eigen::Matrix2f::Identity();
  };
} // namespace nomadz_modeling

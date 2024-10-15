#pragma once

#include <optional>
#include <vector>

#include <Eigen/Core>

#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_configuration/field_dimensions.hpp"

namespace nomadz_modeling {

  struct PerceivedLine {
    // The position of the perceived landmark in robot frame
    nomadz_configuration::Line percept;
    // The covariance of the landmark measurement
    Eigen::Matrix2f cov_percept;
  };

  struct RegisteredLine {
    // The perceived line in robot frame
    nomadz_configuration::Line percept;
    // The original line in world frame
    nomadz_configuration::Line model;
    // The covariance of the line measurement, computed w.r.t. perceptCenter
    Eigen::Matrix2f cov_percept_center;
    // The direction vector of the line relative to the robot (= perceptEnd - perceptStart)
    Eigen::Vector2f percept_direction;
    // The center of the percepted line
    Eigen::Vector2f percept_center;
    // Projection of the robot position onto the line
    Eigen::Vector2f orthogonal_projection;
    // True if the original line is parallel to the global x axis
    bool parallel_to_worl_model_x_axis;
    // The measured angle of the line
    float measured_angle;
    // The alternative angle (-180 degrees)
    float measured_angle_alternative;

    RegisteredLine(nomadz_configuration::Line percept, nomadz_configuration::Line model, Eigen::Matrix2f cov_percept_center);
  };

  class LineRegistrator {

  public:
    LineRegistrator();

    /**
     * Determine which field lines
     * are compatible (given some thresholds) to the assumed robot pose
     * @param robot_pose The assumed robot pose
     * @param perceived_lines A list containing all the perceived lines to analyze
     * @param registered_lines A list to which all compatible measurements are added
     */
    void registerLines(const nomadz_core::Pose2D& robot_pose,
                       const std::vector<PerceivedLine>& perceived_lines,
                       std::vector<RegisteredLine>& registered_lines) const;

  private:
    /**
     * Determine, if the perceived field line matches one of the lines on the field.
     * @param robot_pose The assumed pose of the robot
     * @param perceived_line The percepted line in the robot frame
     * @return An optional containing the line entry in the world model.
     */
    std::optional<nomadz_configuration::Line> getCorrespondingLineInWorldModel(const nomadz_core::Pose2D& robot_pose,
                                                                               const PerceivedLine& perceived_line) const;

    /**
     * Determine the distance of a point to a line segment.
     * @param base The base of the line
     * @param dir The direction of the line
     * @param length The length of the line
     * @param point The point from which the distance to the line is computed
     * @return The distance from the point to the line
     */
    static float getDistanceToLineSegment(const Eigen::Vector2f& base,
                                          const Eigen::Vector2f& dir,
                                          float length,
                                          const Eigen::Vector2f& point);

    // Maximum allowed distance error for line registration
    const float MAX_DISTANCE_ERROR_ = 0.4F;
    // Cov noise added after registration Note(emilio): set to 0 once the measurements have accurate cov
    const Eigen::Matrix2f ADDITIONAL_NOISE_ = 0.05F * Eigen::Matrix2f::Identity();

    std::vector<nomadz_configuration::Line> model_vertical_lines_world_;
    std::vector<nomadz_configuration::Line> model_horizontal_lines_world_;
  };
} // namespace nomadz_modeling

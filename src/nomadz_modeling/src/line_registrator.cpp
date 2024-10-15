#include "nomadz_modeling/line_registrator.hpp"

#include <cmath>

#include "nomadz_core/math/approx.hpp"
#include "nomadz_core/geometry/angle.hpp"
#include "nomadz_core/geometry/projection.hpp"

namespace nomadz_modeling {

  LineRegistrator::LineRegistrator() {

    nomadz_configuration::FieldLines world_model_field_lines;
    for (auto const& field_line : world_model_field_lines.field_lines) {
      nomadz_configuration::Line new_line{field_line.from, field_line.to};
      Eigen::Vector2f new_line_dir = new_line.to - new_line.from;
      const float new_line_length = new_line_dir.norm();
      new_line_dir.normalize();
      if (new_line_length > world_model_field_lines.penalty_mark_size) // Penalty mark is not a line here
      {
        const bool is_vertical = std::abs(new_line_dir.x()) > std::abs(new_line_dir.y());
        if (is_vertical) {
          model_vertical_lines_world_.push_back(new_line);
        } else {
          model_horizontal_lines_world_.push_back(new_line);
        }
      }
    }
  }

  RegisteredLine::RegisteredLine(nomadz_configuration::Line percept,
                                 nomadz_configuration::Line model,
                                 Eigen::Matrix2f cov_percept_center)
      : percept(std::move(percept)), model(std::move(model)), cov_percept_center(std::move(cov_percept_center)) {
    percept_direction = this->percept.to - this->percept.from;
    percept_direction.normalize();
    percept_center = (this->percept.from + this->percept.to) * 0.5F;
    parallel_to_worl_model_x_axis = nomadz_core::approx::isZero(this->model.from.y() - this->model.to.y());
    orthogonal_projection =
      nomadz_core::getOrthogonalProjectionOfPointOnLine(this->percept.from, percept_direction, Eigen::Vector2f::Zero());
    measured_angle = -std::atan2(orthogonal_projection.y(), orthogonal_projection.x());
    measured_angle =
      nomadz_core::angle::normalize(measured_angle + (parallel_to_worl_model_x_axis ? nomadz_core::constants::PI_2 : 0));
    measured_angle_alternative = nomadz_core::angle::normalize(measured_angle - nomadz_core::constants::PI);
  }

  void LineRegistrator::registerLines(const nomadz_core::Pose2D& robot_pose,
                                      const std::vector<PerceivedLine>& perceived_lines,
                                      std::vector<RegisteredLine>& lines) const {

    for (const PerceivedLine& perceived_line : perceived_lines) {
      auto field_line = getCorrespondingLineInWorldModel(robot_pose, perceived_line);
      if (field_line.has_value()) {
        Eigen::Matrix2f cov =
          perceived_line.cov_percept + ADDITIONAL_NOISE_; // Note(emilio): remove added noise once measurement has covariance
        lines.emplace_back(perceived_line.percept, field_line.value(), cov);
      }
    }
  }

  std::optional<nomadz_configuration::Line>
  LineRegistrator::getCorrespondingLineInWorldModel(const nomadz_core::Pose2D& robot_pose,
                                                    const PerceivedLine& perceived_line) const {
    nomadz_configuration::Line percept_line_world{nomadz_core::toAffine2(robot_pose) * perceived_line.percept.from,
                                                  nomadz_core::toAffine2(robot_pose) * perceived_line.percept.to};

    Eigen::Vector2f percept_dir_world = percept_line_world.to - percept_line_world.from;
    const float percept_length = percept_dir_world.norm();
    percept_dir_world.normalize();
    const bool is_vertical = std::abs(percept_dir_world.x()) > std::abs(percept_dir_world.y());
    const std::vector<nomadz_configuration::Line>& model_lines_world =
      is_vertical ? model_vertical_lines_world_ : model_horizontal_lines_world_;
    for (const auto& model_line_world : model_lines_world) {
      Eigen::Vector2f model_dir_world = model_line_world.to - model_line_world.from;
      const float model_length = model_dir_world.norm();
      // A percepted line cannot be longer than the original line
      if (percept_length > 1.25 * model_length) {
        continue;
      }
      model_dir_world.normalize();
      if (getDistanceToLineSegment(model_line_world.from, model_dir_world, model_length, percept_line_world.from) >
            MAX_DISTANCE_ERROR_ ||
          getDistanceToLineSegment(model_line_world.from, model_dir_world, model_length, percept_line_world.to) >
            MAX_DISTANCE_ERROR_) {
        continue;
      }

      return model_line_world;
    }
    return {};
  }

  float LineRegistrator::getDistanceToLineSegment(const Eigen::Vector2f& base,
                                                  const Eigen::Vector2f& dir,
                                                  float length,
                                                  const Eigen::Vector2f& point) {
    float l = (point - base).dot(dir);
    l = std::clamp(l, 0.F, length);
    Eigen::Vector2f projection = base + l * dir;
    return (projection - point).norm();
  }

} // namespace nomadz_modeling

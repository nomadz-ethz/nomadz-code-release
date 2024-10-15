#pragma once

#include <vector>
#include <string>

#include <Eigen/Core>

namespace nomadz_kinematics {
  struct BodyPart {
    std::string name;
    float mass;
    Eigen::Vector3f offset;
    Eigen::Matrix<float, 6, 1> inertia_matrix_lt;
  };

  std::vector<BodyPart> parseMassCalibration(std::string config_file_path);
} // namespace nomadz_kinematics

#include "nomadz_kinematics/mass_calibration_parser.hpp"

#include <yaml-cpp/yaml.h>

namespace nomadz_kinematics {
  std::vector<BodyPart> parseMassCalibration(std::string config_file_path) {
    YAML::Node config = YAML::LoadFile(config_file_path);
    std::vector<BodyPart> body_parts;
    for (const auto& mass_node : config["masses"]) {
      BodyPart body_part;
      body_part.name = mass_node["name"].as<std::string>();
      body_part.mass = mass_node["mass"].as<float>();
      body_part.offset.x() = mass_node["offset"]["x"].as<float>();
      body_part.offset.y() = mass_node["offset"]["y"].as<float>();
      body_part.offset.z() = mass_node["offset"]["z"].as<float>();
      const auto& inertia_node = mass_node["inertia_matrix_lt"];
      for (int i = 0; i < 6; ++i) {
        body_part.inertia_matrix_lt(i) = inertia_node[i].as<float>();
      }

      body_part.mass *= 1e-3;
      body_part.offset *= 1e-3;
      body_part.inertia_matrix_lt *= 1e-9;

      body_parts.push_back(body_part);
    }
    return body_parts;
  }
} // namespace nomadz_kinematics

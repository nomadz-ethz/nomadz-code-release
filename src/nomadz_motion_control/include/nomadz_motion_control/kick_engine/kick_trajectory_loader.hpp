#pragma once

#include <vector>
#include <filesystem>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "nomadz_motion_control/interpolation.hpp"

namespace nomadz_motion_control::kick_engine {

  struct KickPhase {
    InterpolationType interpolation_type;
    float duration;
    std::vector<Eigen::Affine3f> key_frames;
  };

  struct KickTrajectory {
    Eigen::Affine3f starting_position;
    std::vector<KickPhase> kick_phases;
  };

  enum KickType { DEFAULT, NUM_KICK_TYPES };
  const std::map<KickType, std::string> KICK_TYPE_TO_FILE_NAME_MAP = {{KickType::DEFAULT, "defaultKick.yaml"}};

  std::vector<KickTrajectory> loadKickTrajectoriesFromYaml();

  Eigen::Affine3f parseKeyframe(const YAML::Node& trajectory_node);
  KickPhase parsePhase(const YAML::Node& phase_node);
  KickTrajectory parseTrajectory(const YAML::Node& trajectory_node);

  bool isKickTrajectoryValid(const KickTrajectory& kick_trajectory);
} // namespace nomadz_motion_control::kick_engine

#include "nomadz_motion_control/kick_engine/kick_trajectory_loader.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

namespace nomadz_motion_control::kick_engine {
  Eigen::Affine3f parseKeyframe(const YAML::Node& trajectory_node) {
    Eigen::Affine3f key_frame = Eigen::Affine3f::Identity();
    key_frame.translation().x() = trajectory_node["x"].as<float>();
    key_frame.translation().y() = trajectory_node["y"].as<float>();
    key_frame.translation().z() = trajectory_node["z"].as<float>();
    if (trajectory_node["roll"]) {
      key_frame *= Eigen::AngleAxisf(trajectory_node["roll"].as<float>(), Eigen::Vector3f::UnitX());
    }
    if (trajectory_node["pitch"]) {
      key_frame *= Eigen::AngleAxisf(trajectory_node["pitch"].as<float>(), Eigen::Vector3f::UnitY());
    }
    if (trajectory_node["yaw"]) {
      key_frame *= Eigen::AngleAxisf(trajectory_node["yaw"].as<float>(), Eigen::Vector3f::UnitZ());
    }
    return key_frame;
  }

  KickPhase parsePhase(const YAML::Node& phase_node) {
    KickPhase kick_phase;
    kick_phase.duration = phase_node["duration"].as<float>();
    kick_phase.interpolation_type = static_cast<InterpolationType>(phase_node["interpolation_type"].as<int>());

    for (auto key_frame : phase_node["key_frames"]) {
      kick_phase.key_frames.push_back(parseKeyframe(key_frame));
    }
    return kick_phase;
  }

  KickTrajectory parseTrajectory(const YAML::Node& trajectory_node) {
    KickTrajectory kick_trajectory;
    kick_trajectory.starting_position = parseKeyframe(trajectory_node["starting_position"]);
    for (auto kick_phase : trajectory_node["kick_phases"]) {
      YAML::Node phase_node_value = kick_phase.second;
      kick_trajectory.kick_phases.push_back(parsePhase(phase_node_value));
    }
    return kick_trajectory;
  }

  bool isKickTrajectoryValid(const KickTrajectory& kick_trajectory) {
    for (const auto& kick_phase : kick_trajectory.kick_phases) {
      int required_number_of_key_frames = 0;
      switch (kick_phase.interpolation_type) {
      case InterpolationType::LINEAR:
        required_number_of_key_frames = 1;
        break;
      case InterpolationType::QUADRATIC_BEZIER:
        required_number_of_key_frames = 2;
        break;
      case InterpolationType::CUBIC_BEZIER:
        required_number_of_key_frames = 3;
        break;
      default:
        required_number_of_key_frames = -1;
        break;
      }
      if (static_cast<int>(kick_phase.key_frames.size()) != required_number_of_key_frames) {
        return false;
      }
    }
    return true;
  }

  std::vector<KickTrajectory> loadKickTrajectoriesFromYaml() {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("nomadz_motion_control");
    fs::path kick_cfg_directory = fs::path(package_share_directory) / fs::path("config/kicks");

    std::vector<KickTrajectory> kick_trajectories;
    for (int i = 0; i < KickType::NUM_KICK_TYPES; ++i) {
      fs::path kick_cfg_path =
        fs::path(kick_cfg_directory) / fs::path(KICK_TYPE_TO_FILE_NAME_MAP.at(static_cast<KickType>(i)));

      YAML::Node kick_cfg_node = YAML::LoadFile(kick_cfg_path);
      auto kick_trajectory = parseTrajectory(kick_cfg_node);
      if (!isKickTrajectoryValid(kick_trajectory)) {
        throw std::invalid_argument("Invalid combination of interpolation type and key frames numbers");
      }
      kick_trajectories.push_back(kick_trajectory);
    }
    return kick_trajectories;
  }
} // namespace nomadz_motion_control::kick_engine

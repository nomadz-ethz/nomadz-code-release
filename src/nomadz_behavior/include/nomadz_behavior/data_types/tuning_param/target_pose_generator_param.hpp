#pragma once

#include "nomadz_core/geometry/pose.hpp"

#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "nomadz_behavior/data_types/teammates_status.hpp"

namespace fs = std::filesystem;

namespace nomadz_behavior {
  using Positions = std::array<std::array<float, 3>, MAX_NUM_OF_PLAYERS>;

  struct TargetPoseGeneratorParameters {
    // ready pose kicking team
    Positions ready_pose_kicking_team;
    Positions ready_pose_non_kicking_team;

    // distances
    float des_distance_goalkeeper_cover = 0.6F; // [m]
    float distance_to_ball = 0.25F;
    float distance_observation_to_ball = 1.0F;
    float distance_penalty_kick_goalpost = 0.25F;

    // gradient descent
    float scale_gradient = 3.F;
    float step_size_gradient = 0.5F;

    // cover cost
    float cover_scale = 5.F;
    float cover_variance = 1.F; // variance must be small

    // ball CoM cost
    float ball_CoM_scale = 100.F; // conservative

    // avoidance cost
    float avoidance_scale_opponent = 200.F;
    float avoidance_variance_opponent = 0.5F;
    float avoidance_scale_team = 100.F;
    float avoidance_variance_team = 0.5F;

    // goal free cost
    float goal_free_scale = 500.0;

    // offense defense cost
    float offense_defense_scale = 1700.F;
    float offense_defense_variance = 0.8F;
    float offense_defense_x_switch_behavior = 0.0F;
  };

  inline TargetPoseGeneratorParameters loadTargetPoseGeneratorParameters() {
    // const Behavior behavior;
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("nomadz_behavior");
    YAML::Node yaml_file =
      YAML::LoadFile(fs::path(package_share_directory) / fs::path("config") / "target_pose_generator.yaml");

    TargetPoseGeneratorParameters parameters;

    parameters.ready_pose_kicking_team = yaml_file["ready_pose"]["kicking_team"].as<Positions>();
    parameters.ready_pose_non_kicking_team = yaml_file["ready_pose"]["non_kicking_team"].as<Positions>();

    const auto distance_parameters = yaml_file["desired_distances"];
    parameters.des_distance_goalkeeper_cover = distance_parameters["des_distance_goal"].as<float>();
    parameters.distance_to_ball = distance_parameters["to_ball"].as<float>();
    parameters.distance_observation_to_ball = distance_parameters["observation_ball"].as<float>();
    parameters.distance_penalty_kick_goalpost = distance_parameters["penalty_kick_goalpost"].as<float>();

    const auto optimization_parameters = yaml_file["target_pose_optimization_parameters"];
    parameters.scale_gradient = optimization_parameters["scale_gradient"].as<float>();
    parameters.step_size_gradient = optimization_parameters["step_size_gradient"].as<float>();
    parameters.cover_scale = optimization_parameters["cover_scale"].as<float>();
    parameters.cover_variance = optimization_parameters["cover_variance"].as<float>();
    parameters.ball_CoM_scale = optimization_parameters["ball_CoM_scale"].as<float>();
    parameters.avoidance_scale_opponent = optimization_parameters["avoidance_scale_opponent"].as<float>();
    parameters.avoidance_variance_opponent = optimization_parameters["avoidance_variance_opponent"].as<float>();
    parameters.avoidance_scale_team = optimization_parameters["avoidance_scale_team"].as<float>();
    parameters.avoidance_variance_team = optimization_parameters["avoidance_variance_team"].as<float>();
    parameters.goal_free_scale = optimization_parameters["goal_free_scale"].as<float>();
    parameters.offense_defense_scale = optimization_parameters["offense_defense_scale"].as<float>();
    parameters.offense_defense_variance = optimization_parameters["offense_defense_variance"].as<float>();
    parameters.offense_defense_x_switch_behavior = optimization_parameters["offense_defense_x_switch_behavior"].as<float>();

    return parameters;
  }

} // namespace nomadz_behavior

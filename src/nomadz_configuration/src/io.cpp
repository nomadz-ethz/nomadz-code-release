#include "nomadz_configuration/io.hpp"

#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

namespace nomadz_configuration {
  FieldDimensions getLocationFieldDimensions(const std::string& location) {
    const fs::path locations_dir_path =
      fs::path(ament_index_cpp::get_package_share_directory("nomadz_configuration")) / "config" / "locations";
    const fs::path field_dimensions_file_path = locations_dir_path / location / "field_dimensions.yaml";
    return loadFromFile<FieldDimensions>(field_dimensions_file_path);
  }

  GameSettings getGameSettings() {
    const fs::path config_dir_path = fs::path(ament_index_cpp::get_package_share_directory("nomadz_configuration"));
    const fs::path game_settings_path = config_dir_path / "config" / "game_settings.yaml";
    return loadFromFile<GameSettings>(game_settings_path);
  }
} // namespace nomadz_configuration

namespace YAML {

  bool convert<nomadz_configuration::FieldDimensions>::decode(const Node& node,
                                                              nomadz_configuration::FieldDimensions& field_dimensions) {

    field_dimensions.field_length = node["field_length"].as<float>();
    field_dimensions.field_width = node["field_width"].as<float>();
    field_dimensions.line_width = node["line_width"].as<float>();
    field_dimensions.penalty_mark_size = node["penalty_mark_size"].as<float>();
    field_dimensions.goal_area_length = node["goal_area_length"].as<float>();
    field_dimensions.goal_area_width = node["goal_area_width"].as<float>();
    field_dimensions.penalty_area_length = node["penalty_area_length"].as<float>();
    field_dimensions.penalty_area_width = node["penalty_area_width"].as<float>();
    field_dimensions.penalty_mark_distance = node["penalty_mark_distance"].as<float>();
    field_dimensions.center_circle_diameter = node["center_circle_diameter"].as<float>();
    field_dimensions.border_strip_width = node["border_strip_width"].as<float>();
    field_dimensions.goal_width = node["goal_width"].as<float>();
    field_dimensions.goal_depth = node["goal_depth"].as<float>();
    field_dimensions.goal_height = node["goal_height"].as<float>();
    field_dimensions.goal_post_diameter = node["goal_post_diameter"].as<float>();

    return true;
  }

  bool
  convert<nomadz_configuration::BallSpecification>::decode(const Node& node,
                                                           nomadz_configuration::BallSpecification& ball_specification) {
    ball_specification.ball_radius = node["ball_radius"].as<float>();
    ball_specification.ball_friction = node["ball_friction"].as<float>();

    return true;
  }

  bool convert<nomadz_configuration::GameSettings>::decode(const Node& node,
                                                           nomadz_configuration::GameSettings& game_settings) {
    game_settings.team_id = node["team_id"].as<int>();
    game_settings.team_color = node["team_color"].as<std::string>();
    game_settings.keeper_color = node["keeper_color"].as<std::string>();
    game_settings.player_id = node["player_id"].as<int>();
    game_settings.player_role = node["player_role"].as<std::string>();
    game_settings.location = node["location"].as<std::string>();

    return true;
  }
} // namespace YAML

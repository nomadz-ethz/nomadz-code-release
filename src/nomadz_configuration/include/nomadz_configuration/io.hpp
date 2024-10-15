#pragma once

#include <fstream>
#include <filesystem>

#include <yaml-cpp/yaml.h>

#include "nomadz_configuration/field_dimensions.hpp"
#include "nomadz_configuration/ball_specification.hpp"
#include "nomadz_configuration/game_settings.hpp"

namespace nomadz_configuration {

  // generic function to load from file
  template <typename T> inline T loadFromFile(const std::filesystem::path& file_path) {
    std::ifstream ifs(file_path);
    YAML::Node config_yaml = YAML::Load(ifs);
    return config_yaml.as<T>();
  }

  constexpr const char* DEFAULT_LOCATION = "default";

  FieldDimensions getLocationFieldDimensions(const std::string& location = DEFAULT_LOCATION);

  GameSettings getGameSettings();

}; // namespace nomadz_configuration

namespace YAML {

  template <> struct convert<nomadz_configuration::FieldDimensions> {
    static bool decode(const Node& node, nomadz_configuration::FieldDimensions& field_dimensions);
  };

  template <> struct convert<nomadz_configuration::BallSpecification> {
    static bool decode(const Node& node, nomadz_configuration::BallSpecification& ball_specification);
  };

  template <> struct convert<nomadz_configuration::GameSettings> {
    static bool decode(const Node& node, nomadz_configuration::GameSettings& game_settings);
  };
} // namespace YAML

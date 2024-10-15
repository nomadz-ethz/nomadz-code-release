#pragma once

#include <Eigen/Core>
#include <vector>

namespace nomadz_configuration {

  /**
   * @brief Field dimensions (in meters)
   *
   * The default values are based on the RoboCup SPL 2024 rules.
   */
  struct FieldDimensions {
    float field_length = 9.F;
    float field_width = 6.F;
    float line_width = 0.05F;
    float penalty_mark_size = 0.1F;
    float goal_area_length = 0.6F;
    float goal_area_width = 2.2F;
    float penalty_area_length = 1.65F;
    float penalty_area_width = 4.F;
    float penalty_mark_distance = 1.3F;
    float center_circle_diameter = 1.5F;
    float border_strip_width = 0.7F;
    float goal_width = 1.5F;
    float goal_depth = 0.5F;
    float goal_height = 0.8F;
    float goal_post_diameter = 0.1F;
    float center_circle_radius = center_circle_diameter / 2.F;
  };

  struct Line {
    Eigen::Vector2f from;
    Eigen::Vector2f to;
  };

  // TODO(emilio): define Dims based on FieldDimensions
  struct Dims {
    float x_pos_opponent_field_border = 5.200F; // origin -> far field boundary
    float x_pos_opponent_goal = 5.055F;         // origin -> middle far side of goal,
    float x_pos_opponent_goal_post = 4.525F;    // origin -> middle goal post
    float x_pos_opponent_ground_line = 4.500F;  // origin -> middle goal line
    float x_pos_opponent_goal_area = 3.900F;    // origin -> middle near goal area line
    float x_pos_opponent_penalty_mark = 3.200F; // origin -> middle penalty mark
    float x_pos_opponent_penalty_area = 2.850F; // origin -> middle near penalty area line
    float x_pos_penalty_striker_position = x_pos_opponent_penalty_area;
    float x_pos_halfway_line = 0;
    float x_pos_own_penalty_area = -x_pos_opponent_penalty_area;
    float x_pos_own_penalty_mark = -x_pos_opponent_penalty_mark;
    float x_pos_own_goal_area = -x_pos_opponent_goal_area;
    float x_pos_own_ground_line = -x_pos_opponent_ground_line;
    float x_pos_own_goal_post = -x_pos_opponent_goal_post;
    float x_pos_own_goal = -x_pos_opponent_goal;
    float x_pos_own_field_border = -x_pos_opponent_field_border;

    float y_pos_left_field_border = 3.700F; // origin -> side field boundary
    float y_pos_left_sideline = 3.000F;     // origin -> middle side line
    float y_pos_left_penalty_area = 2.000F; // origin -> middle left penalty area line
    float y_pos_left_goal_area = 1.100F;    // origin -> middle left goal area line
    float y_pos_left_goal = 0.800F;         // origin -> middle left goal post
    float y_pos_right_goal = -y_pos_left_goal;
    float y_pos_right_goal_area = -y_pos_left_goal_area;
    float y_pos_right_penalty_area = -y_pos_left_penalty_area;
    float y_pos_right_sideline = -y_pos_left_sideline;
    float y_pos_right_field_border = -y_pos_left_field_border;

    float field_lines_width = 0.050F;
    float center_circle_radius = 0.750F;
    float goal_post_radius = 0.050F;
    float cross_bar_radius = goal_post_radius;
    float goal_height = 0.900F;
    float penalty_mark_size = 0.100F;
  };

  // NOTE(@naefjo): copy paste straight from bhuman `FieldDimensions.cfg`
  struct FieldLines : public Dims {

    std::vector<Line> field_lines = {

      // field border lines
      {{x_pos_opponent_ground_line, y_pos_right_sideline}, {x_pos_opponent_ground_line, y_pos_left_sideline}},
      {{x_pos_opponent_ground_line, y_pos_left_sideline}, {x_pos_own_ground_line, y_pos_left_sideline}},
      {{x_pos_own_ground_line, y_pos_left_sideline}, {x_pos_own_ground_line, y_pos_right_sideline}},
      {{x_pos_own_ground_line, y_pos_right_sideline}, {x_pos_opponent_ground_line, y_pos_right_sideline}},

      // center line
      {{x_pos_halfway_line, y_pos_left_sideline}, {x_pos_halfway_line, y_pos_right_sideline}},

      // goal areas
      {{x_pos_own_ground_line, y_pos_left_goal_area}, {x_pos_own_goal_area, y_pos_left_goal_area}},
      {{x_pos_own_goal_area, y_pos_left_goal_area}, {x_pos_own_goal_area, y_pos_right_goal_area}},
      {{x_pos_own_goal_area, y_pos_right_goal_area}, {x_pos_own_ground_line, y_pos_right_goal_area}},

      {{x_pos_opponent_ground_line, y_pos_left_goal_area}, {x_pos_opponent_goal_area, y_pos_left_goal_area}},
      {{x_pos_opponent_goal_area, y_pos_left_goal_area}, {x_pos_opponent_goal_area, y_pos_right_goal_area}},
      {{x_pos_opponent_goal_area, y_pos_right_goal_area}, {x_pos_opponent_ground_line, y_pos_right_goal_area}},

      // penalty areas
      {{x_pos_own_ground_line, y_pos_left_penalty_area}, {x_pos_own_penalty_area, y_pos_left_penalty_area}},
      {{x_pos_own_penalty_area, y_pos_left_penalty_area}, {x_pos_own_penalty_area, y_pos_right_penalty_area}},
      {{x_pos_own_penalty_area, y_pos_right_penalty_area}, {x_pos_own_ground_line, y_pos_right_penalty_area}},

      {{x_pos_opponent_ground_line, y_pos_left_penalty_area}, {x_pos_opponent_penalty_area, y_pos_left_penalty_area}},
      {{x_pos_opponent_penalty_area, y_pos_left_penalty_area}, {x_pos_opponent_penalty_area, y_pos_right_penalty_area}},
      {{x_pos_opponent_penalty_area, y_pos_right_penalty_area}, {x_pos_opponent_ground_line, y_pos_right_penalty_area}},

      // penalty and center marks
      {{3.150F, 0.0F}, {3.250F, 0.0F}},
      {{x_pos_opponent_penalty_mark, -field_lines_width}, {x_pos_opponent_penalty_mark, field_lines_width}},

      {{-3.150F, 0.0F}, {-3.250F, 0.0F}},
      {{x_pos_own_penalty_mark, -field_lines_width}, {x_pos_own_penalty_mark, field_lines_width}},
      {{-field_lines_width, 0.0F}, {field_lines_width, 0.0F}}};
  };

  // NOTE(emilio): copy paste straight from bhuman `FieldDimensions.cfg`
  struct Corners : public Dims {
    const std::vector<Eigen::Vector2f> X_CORNER = {{x_pos_halfway_line, center_circle_radius},
                                                   {x_pos_halfway_line, -center_circle_radius}};

    const std::vector<Eigen::Vector2f> T_CORNER_0 = {{x_pos_halfway_line, center_circle_radius},
                                                     {x_pos_halfway_line, -center_circle_radius},
                                                     {x_pos_own_ground_line, y_pos_left_penalty_area},
                                                     {x_pos_own_ground_line, y_pos_right_penalty_area},
                                                     {x_pos_own_ground_line, y_pos_left_goal_area},
                                                     {x_pos_own_ground_line, y_pos_right_goal_area}};

    const std::vector<Eigen::Vector2f> T_CORNER_90 = {{x_pos_halfway_line, center_circle_radius},
                                                      {x_pos_halfway_line, -center_circle_radius},
                                                      {x_pos_halfway_line, y_pos_right_sideline}};

    const std::vector<Eigen::Vector2f> T_CORNER_180 = {{x_pos_halfway_line, center_circle_radius},
                                                       {x_pos_halfway_line, -center_circle_radius},
                                                       {x_pos_opponent_ground_line, y_pos_left_penalty_area},
                                                       {x_pos_opponent_ground_line, y_pos_right_penalty_area},
                                                       {x_pos_opponent_ground_line, y_pos_left_goal_area},
                                                       {x_pos_opponent_ground_line, y_pos_right_goal_area}};

    const std::vector<Eigen::Vector2f> T_CORNER_270 = {{x_pos_halfway_line, center_circle_radius},
                                                       {x_pos_halfway_line, -center_circle_radius},
                                                       {x_pos_halfway_line, y_pos_left_sideline}};

    const std::vector<Eigen::Vector2f> L_CORNER_0 = {{x_pos_halfway_line, center_circle_radius},
                                                     {x_pos_halfway_line, -center_circle_radius},
                                                     {x_pos_own_ground_line, y_pos_left_penalty_area},
                                                     {x_pos_own_ground_line, y_pos_right_penalty_area},
                                                     {x_pos_own_ground_line, y_pos_left_goal_area},
                                                     {x_pos_own_ground_line, y_pos_right_goal_area},
                                                     {x_pos_halfway_line, y_pos_right_sideline},
                                                     {x_pos_own_ground_line, y_pos_right_sideline},
                                                     {x_pos_opponent_penalty_area, y_pos_right_penalty_area},
                                                     {x_pos_opponent_goal_area, y_pos_right_goal_area}};

    const std::vector<Eigen::Vector2f> L_CORNER_90 = {{x_pos_halfway_line, center_circle_radius},
                                                      {x_pos_halfway_line, -center_circle_radius},
                                                      {x_pos_opponent_ground_line, y_pos_left_penalty_area},
                                                      {x_pos_opponent_ground_line, y_pos_right_penalty_area},
                                                      {x_pos_opponent_ground_line, y_pos_left_goal_area},
                                                      {x_pos_opponent_ground_line, y_pos_right_goal_area},
                                                      {x_pos_halfway_line, y_pos_right_sideline},
                                                      {x_pos_opponent_ground_line, y_pos_right_sideline},
                                                      {x_pos_own_penalty_area, y_pos_right_penalty_area},
                                                      {x_pos_own_goal_area, y_pos_right_goal_area}};

    const std::vector<Eigen::Vector2f> L_CORNER_180 = {{x_pos_halfway_line, center_circle_radius},
                                                       {x_pos_halfway_line, -center_circle_radius},
                                                       {x_pos_opponent_ground_line, y_pos_left_penalty_area},
                                                       {x_pos_opponent_ground_line, y_pos_right_penalty_area},
                                                       {x_pos_opponent_ground_line, y_pos_left_goal_area},
                                                       {x_pos_opponent_ground_line, y_pos_right_goal_area},
                                                       {x_pos_halfway_line, y_pos_left_sideline},
                                                       {x_pos_opponent_ground_line, y_pos_left_sideline},
                                                       {x_pos_own_penalty_area, y_pos_left_penalty_area},
                                                       {x_pos_own_goal_area, y_pos_left_goal_area}};

    const std::vector<Eigen::Vector2f> L_CORNER_270 = {{x_pos_halfway_line, center_circle_radius},
                                                       {x_pos_halfway_line, -center_circle_radius},
                                                       {x_pos_own_ground_line, y_pos_left_penalty_area},
                                                       {x_pos_own_ground_line, y_pos_right_penalty_area},
                                                       {x_pos_own_ground_line, y_pos_left_goal_area},
                                                       {x_pos_own_ground_line, y_pos_right_goal_area},
                                                       {x_pos_halfway_line, y_pos_left_sideline},
                                                       {x_pos_own_ground_line, y_pos_left_sideline},
                                                       {x_pos_opponent_penalty_area, y_pos_left_penalty_area},
                                                       {x_pos_opponent_goal_area, y_pos_left_goal_area}};
  };

} // namespace nomadz_configuration

#include "nomadz_configuration/field_info.hpp"

namespace nomadz_configuration {

  Eigen::AlignedBox2f computefieldBoundingBox(const FieldDimensions& field_dimensions) {
    const Eigen::Vector2f bl_corner =
      Eigen::Vector2f(-field_dimensions.field_length / 2.F - field_dimensions.border_strip_width,
                      -field_dimensions.field_width / 2.F - field_dimensions.border_strip_width);
    const Eigen::Vector2f tr_corner =
      Eigen::Vector2f(field_dimensions.field_length / 2.F + field_dimensions.border_strip_width,
                      field_dimensions.field_width / 2.F + field_dimensions.border_strip_width);

    return {bl_corner, tr_corner};
  }

} // namespace nomadz_configuration

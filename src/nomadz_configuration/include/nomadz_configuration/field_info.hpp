#pragma once

#include <Eigen/Geometry>

#include "nomadz_configuration/field_dimensions.hpp"

namespace nomadz_configuration {

  Eigen::AlignedBox2f computefieldBoundingBox(const FieldDimensions& field_dimensions);

} // namespace nomadz_configuration

/**
 * @file GlobalFieldCoverage.h
 *
 * Definition of a class representing the global field coverage among all robots.
 * A part of a field is considered to be 'covered' if at least one robot looks at it.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Felix Wenk
 */

#pragma once

#include "Representations/Modeling/FieldCoverage.h"

#include "Core/Streams/FieldWrapper.h"

STREAMABLE_DECLARE_LEGACY(GlobalFieldCoverage)
STREAMABLE_DECLARE_LEGACY(Grid)

STREAMABLE_LEGACY(GlobalFieldCoverage, {
  public : STREAMABLE_LEGACY(Grid,
                             {
                             public:
                               unsigned char coverage(int index, unsigned time) const;
                               void setCoverage(int index, unsigned time, unsigned char coverage),

                                 /* cells[i] is the timestamp when the cell has been looked at the last time by one robot. */
                                 FIELD_WRAPPER_DEFAULT_LEGACY(
                                   unsigned[FieldCoverage::GridInterval::xSteps * FieldCoverage::GridInterval::ySteps],
                                   nomadz_msgs::msg::Grid::cells,
                                   cells),

                                 // Initialization
                                 memset(cells, 0, sizeof(cells));
                             }),
  FIELD_WRAPPER_DEFAULT_LEGACY(Grid, nomadz_msgs::msg::GlobalFieldCoverage::grid, grid),
  FIELD_WRAPPER_LEGACY(int, 0, nomadz_msgs::msg::GlobalFieldCoverage::worst_covered_cell_index, worstCoveredCellIndex),
  FIELD_WRAPPER_LEGACY(int,
                       0,
                       nomadz_msgs::msg::GlobalFieldCoverage::threshold,
                       threshold), /**< The coverage value above which a cell is considered to be covered. */
  FIELD_WRAPPER_DEFAULT_LEGACY(Vector2<>,
                               nomadz_msgs::msg::GlobalFieldCoverage::patrol_target,
                               patrolTarget), /**< Target the robot should try to approach while patrolling. */
  FIELD_WRAPPER_LEGACY(bool,
                       false,
                       nomadz_msgs::msg::GlobalFieldCoverage::patrol_target_valid,
                       patrolTargetValid), /**< True if the patrol target is valid, otherwise false. */
});

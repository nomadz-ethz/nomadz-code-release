/**
 * @file FieldCoverage.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Felix Wenk
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Math/Vector2.h"
#include <cstring>

#include "Core/Streams/FieldWrapper.h"

STREAMABLE_DECLARE_LEGACY(FieldCoverage)
STREAMABLE_DECLARE_LEGACY(GridInterval)
STREAMABLE_DECLARE_LEGACY(Target)

STREAMABLE_LEGACY(FieldCoverage, {
public:
  STREAMABLE_LEGACY(GridInterval, {
  public:
    static const int xSteps = 12;
    static const int ySteps = 8;
    static const int intervals = 3; /* Number of intervals into the coverage grid is divided up. */
    static const int intervalSize = xSteps * ySteps / intervals;
    static const int maxCoverage = 255;
    static const unsigned tick = 300; /* Milliseconds one coverage tick is worth. */

    static bool assertIntervalSize() { return xSteps * ySteps % intervals == 0; }
    void nextInterval() { interval = (interval + 1) % intervals; }

    static Vector2<int> index2CellCoordinates(int idx) { return Vector2<int>(idx / ySteps, idx % ySteps); }
    static int cellCoordinates2Index(unsigned x, unsigned y) { return x * ySteps + y; }
    ,
      FIELD_WRAPPER_LEGACY(
        unsigned,
        0,
        nomadz_msgs::msg::GridInterval::timestamp,
        timestamp), /* The timestamp when this grid has been updated. The coverage values are relative to this timestamp. */
                    /* Interval of the grid currently represented.
                     * Interval i represents cells from [i * (xSteps*ySteps/intervals), (i+1) * (xSteps*ySteps/intervals) - 1].
                     * interval < intervals must always hold. */
      FIELD_WRAPPER_LEGACY(unsigned char, 0, nomadz_msgs::msg::GridInterval::interval, interval),
      FIELD_WRAPPER_DEFAULT_LEGACY(unsigned char[intervalSize], nomadz_msgs::msg::GridInterval::cells, cells),

      // Initialization
      memset(cells, 0, xSteps * ySteps / intervals);
  });

  STREAMABLE_LEGACY(Target, {
  public:
    Target(float x, float y, unsigned short coverage = 0, bool valid = true);
    Target(const Vector2<>& target, unsigned short coverage = 0, bool valid = true),
      FIELD_WRAPPER_DEFAULT_LEGACY(Vector2<>, nomadz_msgs::msg::Target::target, target),
      FIELD_WRAPPER_LEGACY(unsigned short, 0, nomadz_msgs::msg::Target::coverage, coverage),
      FIELD_WRAPPER_LEGACY(
        bool,
        false,
        nomadz_msgs::msg::Target::is_valid,
        isValid), /* True if this is a valid target. The coordinates and coverage should be ignored if valid is false. */
  });

  unsigned char coverage(int cellIdx, unsigned time) const,
    (Target)worstNoTurnHalfRangeTarget, /* Worst covered cell in relative coordinates which is closer than the half the
                                                                                       fieldcoverage range. */
    (Target)worstNoTurnRangeTarget, /* Worst covered cell in relative coordinates which is 'close' and for which you do not
                                                                                   have to turn around. */
    (Target)worstNoTurnTarget, /* Worst covered cell on the field in relative coordinates for which you do not have to turn
                                                                              around. */
    (Target)worstTarget,       /* Worst covered cell on the field in field coordinates. */
    FIELD_WRAPPER_DEFAULT_LEGACY(unsigned[GridInterval::xSteps * GridInterval::ySteps],
                                 nomadz_msgs::msg::FieldCoverage::cells,
                                 cells), /* cells[i] contains the last-seen timestamp of the i-th cell. */
    FIELD_WRAPPER_LEGACY(float, 0, nomadz_msgs::msg::FieldCoverage::mean, mean), /* The mean coverage value. */
    FIELD_WRAPPER_LEGACY(
      float, 0, nomadz_msgs::msg::FieldCoverage::stddev, stddev), /* The variance of the coverage values.. */
    FIELD_WRAPPER_LEGACY(bool,
                         false,
                         nomadz_msgs::msg::FieldCoverage::throw_in,
                         throwIn), /* True if the field coverage has been modified because the ball has been thrown in. */
});

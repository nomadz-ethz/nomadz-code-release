/**
 * @file OdometryCorrectionTable.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Settings.h"
#include "Core/Streams/InStreams.h"
#include "Core/Math/Pose2D.h"
#include <string>

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/correction_table_entry1_d.hpp"
#endif
STREAMABLE_DECLARE(CorrectionTableEntry1D)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/odometry_correction_table.hpp"
#endif
STREAMABLE_DECLARE(OdometryCorrectionTable)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/correction_table_entry2_d.hpp"
#endif
STREAMABLE_DECLARE(CorrectionTableEntry2D)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/odometry_correction_table2_d.hpp"
#endif
STREAMABLE_DECLARE(OdometryCorrectionTable2D)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/odometry_correction_tables.hpp"
#endif
STREAMABLE_DECLARE(OdometryCorrectionTables)

STREAMABLE_ROS(OdometryCorrectionTable, {
  STREAMABLE_ROS(CorrectionTableEntry1D,
                 {
                   ,
                   FIELD_WRAPPER(float, 400.f, nomadz_msgs::msg::CorrectionTableEntry1D::speed, speed),
                   FIELD_WRAPPER(float, 1.f, nomadz_msgs::msg::CorrectionTableEntry1D::multiplier, multiplier),
                 });

  using Entry = CorrectionTableEntry1D;
  ,
    FIELD_WRAPPER_DEFAULT(
      std::vector<CorrectionTableEntry1D>, nomadz_msgs::msg::OdometryCorrectionTable::odometry_table, odometryTable),
});

STREAMABLE_ROS(OdometryCorrectionTable2D, {
  STREAMABLE_ROS(CorrectionTableEntry2D,
                 {
                   ,
                   FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::CorrectionTableEntry2D::speed_x, speedX),
                   FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::CorrectionTableEntry2D::speed_r, speedR),
                   FIELD_WRAPPER(float, 1.f, nomadz_msgs::msg::CorrectionTableEntry2D::multiplier, multiplier),
                 });

  using Entry = CorrectionTableEntry2D;
  ,
    FIELD_WRAPPER_DEFAULT(
      std::vector<CorrectionTableEntry2D>, nomadz_msgs::msg::OdometryCorrectionTable2D::odometry_table, odometryTable),
});

STREAMABLE_ROS(OdometryCorrectionTables,
               {
                 ,
                 FIELD_WRAPPER_DEFAULT(OdometryCorrectionTable,
                                       nomadz_msgs::msg::OdometryCorrectionTables::back_correction_table_preview,
                                       backCorrectionTablePreview),
                 FIELD_WRAPPER_DEFAULT(OdometryCorrectionTable,
                                       nomadz_msgs::msg::OdometryCorrectionTables::forward_correction_table_preview,
                                       forwardCorrectionTablePreview),
                 FIELD_WRAPPER_DEFAULT(OdometryCorrectionTable,
                                       nomadz_msgs::msg::OdometryCorrectionTables::side_correction_table_preview,
                                       sideCorrectionTablePreview),
                 FIELD_WRAPPER_DEFAULT(OdometryCorrectionTable,
                                       nomadz_msgs::msg::OdometryCorrectionTables::back_correction_table,
                                       backCorrectionTable),
                 FIELD_WRAPPER_DEFAULT(OdometryCorrectionTable,
                                       nomadz_msgs::msg::OdometryCorrectionTables::forward_correction_table,
                                       forwardCorrectionTable),
                 FIELD_WRAPPER_DEFAULT(OdometryCorrectionTable,
                                       nomadz_msgs::msg::OdometryCorrectionTables::side_correction_table,
                                       sideCorrectionTable),
                 FIELD_WRAPPER_DEFAULT(OdometryCorrectionTable,
                                       nomadz_msgs::msg::OdometryCorrectionTables::rot_correction_table,
                                       rotCorrectionTable),
                 FIELD_WRAPPER_DEFAULT(OdometryCorrectionTable2D,
                                       nomadz_msgs::msg::OdometryCorrectionTables::rot2_d_correction_table,
                                       rot2DCorrectionTable),
               });

class OdometryCorrection {
public:
  static float correct1D(const OdometryCorrectionTable& table, const float& odometryAbs, const float& absExecutedSpeed) {
    float result = 1.f;
    if (odometryAbs < 0.001f)
      return result;
    const int entries = static_cast<int>(table.odometryTable.size());
    int entry = 0, lastEntry = 0;
    while (entry < (entries - 1) && absExecutedSpeed >= table.odometryTable[entry].speed)
      entry++;
    lastEntry = std::max(0, entry - 1);

    float regStartSpeed;
    float regEndSpeed;
    float speedDiff;
    regStartSpeed = table.odometryTable[lastEntry].speed;
    regEndSpeed = table.odometryTable[entry].speed;
    speedDiff = std::abs(regEndSpeed - regStartSpeed);
    if (speedDiff > 0) {
      float factor = std::min<float>((std::abs(regEndSpeed - absExecutedSpeed)) / speedDiff, 1.f);
      result = table.odometryTable[lastEntry].multiplier * (factor) + table.odometryTable[entry].multiplier * (1 - factor);
    } else
      result = table.odometryTable[lastEntry].multiplier;
    return result;
  }

  static float correct2D(const OdometryCorrectionTable2D& table,
                         const float& odometryAbs,
                         const float& absExecutedSpeed,
                         const float& absExecutedRot) {
    float result = 1.f;
    if (odometryAbs < 0.001f)
      return result;
    const int entries = static_cast<int>(table.odometryTable.size());
    int entry = 0;
    int entryLowSpeed = 0;
    int entryHighSpeed = 0;
    float regStartSpeedX = 0;
    float regEndSpeedX = 0;
    float speedDiffX = 0;

    while (entry < (entries - 1) && absExecutedSpeed >= table.odometryTable[entry].speedX)
      entry++;
    entry = std::max(entry - 1, 0);
    entryLowSpeed = entry;
    while (entryLowSpeed > 0 && absExecutedRot <= table.odometryTable[entryLowSpeed].speedR)
      entryLowSpeed--;
    entryHighSpeed = std::min(entries - 1, entry + 1);
    while (entryHighSpeed < (entries - 1) && absExecutedRot <= table.odometryTable[entryHighSpeed].speedR)
      entryHighSpeed++;

    regStartSpeedX = table.odometryTable[entryLowSpeed].speedX;
    regEndSpeedX = table.odometryTable[entryHighSpeed].speedX;
    speedDiffX = std::abs(regEndSpeedX - regStartSpeedX);
    float rFactor = 1.f;
    if (speedDiffX > 0)
      rFactor = (std::abs(regEndSpeedX - absExecutedSpeed)) / speedDiffX;
    else
      rFactor = table.odometryTable[entryLowSpeed].multiplier;
    float factor = std::min(rFactor, 1.f);
    result = table.odometryTable[entryLowSpeed].multiplier * (factor) +
             table.odometryTable[entryHighSpeed].multiplier * (1 - factor);

    return result;
  }

  static Pose2D correctPreview(const Pose2D& speed,
                               const Pose2D& odometry,
                               const OdometryCorrectionTable& backCorrectionPreview,
                               const OdometryCorrectionTable& forwardCorrectionPreview,
                               const OdometryCorrectionTable& sideCorrectionPreview) {
    float xCorrectionFactor, yCorrectionFactor, rotCorrectionFactor;
    xCorrectionFactor = yCorrectionFactor = rotCorrectionFactor = 1.f;
    Pose2D correctedOdometry;
    // OdometryCorrection
    if (speed.translation.x < 0) {
      xCorrectionFactor = correct1D(backCorrectionPreview, std::abs(odometry.translation.x), std::abs(speed.translation.x));
    } else if (speed.translation.x > 0) {
      xCorrectionFactor =
        correct1D(forwardCorrectionPreview, std::abs(odometry.translation.x), std::abs(speed.translation.x));
    }
    if (speed.translation.y != 0) {
      yCorrectionFactor = correct1D(sideCorrectionPreview, std::abs(odometry.translation.y), std::abs(speed.translation.y));
    }

    correctedOdometry.translation.x = odometry.translation.x * xCorrectionFactor;
    correctedOdometry.translation.y = odometry.translation.y * yCorrectionFactor;
    correctedOdometry.rotation = odometry.rotation * rotCorrectionFactor;
    return correctedOdometry;
  }

  static Pose2D correct(const Pose2D& speed,
                        const Pose2D& odometry,
                        const OdometryCorrectionTable& backCorrection,
                        const OdometryCorrectionTable& forwardCorrection,
                        const OdometryCorrectionTable& sideCorrection,
                        const OdometryCorrectionTable& rotCorrection,
                        const OdometryCorrectionTable2D& rot2DCorrection) {
    float xCorrectionFactor, yCorrectionFactor, rotCorrectionFactor;
    xCorrectionFactor = yCorrectionFactor = rotCorrectionFactor = 1.f;
    Pose2D correctedOdometry;
    // OdometryCorrection
    if (speed.translation.x < 0) {
      xCorrectionFactor = correct1D(backCorrection, std::abs(odometry.translation.x), std::abs(speed.translation.x));
    } else if (speed.translation.x > 0) {
      xCorrectionFactor = correct1D(forwardCorrection, std::abs(odometry.translation.x), std::abs(speed.translation.x));
    }
    if (speed.translation.y != 0) {
      yCorrectionFactor = correct1D(sideCorrection, std::abs(odometry.translation.y), std::abs(speed.translation.y));
    }
    if (speed.translation.x == 0 && speed.rotation != 0) {
      rotCorrectionFactor = correct1D(rotCorrection, std::abs(odometry.rotation), std::abs(speed.rotation));
    } else if (speed.rotation != 0) {
      rotCorrectionFactor =
        correct2D(rot2DCorrection, std::abs(odometry.rotation), std::abs(speed.translation.x), std::abs(speed.rotation));
    }

    correctedOdometry.translation.x = odometry.translation.x * xCorrectionFactor;
    correctedOdometry.translation.y = odometry.translation.y * yCorrectionFactor;
    correctedOdometry.rotation = odometry.rotation * rotCorrectionFactor;
    return correctedOdometry;
  }
};

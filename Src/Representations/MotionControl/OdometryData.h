/**
 * @file OdometryData.h
 *
 * Contains the OdometryData class.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Max Risler
 */

#pragma once

#include "Core/Math/Pose2D.h"
#include "Core/Streams/AutoStreamable.h"

/**
 * @class OdometryData
 * OdometryData contains an approximation of overall movement the robot has done.
 * @attention Only use differences of OdometryData at different times.
 * Position in mm
 */
STREAMABLE_ALIAS(OdometryData, Pose2D, {});

/**
 * @class GroundTruthOdometryData
 * Contains an observed overall movement the robot has done.
 */
STREAMABLE_ALIAS(GroundTruthOdometryData, OdometryData, {});

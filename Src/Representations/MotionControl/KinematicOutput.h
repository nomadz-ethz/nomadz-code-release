/**
 * @file KinematicOutput.h
 *
 * This file declares a class that represents the output of modules generating motion.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is  <A href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</A>
 */

#pragma once

#include "Representations/Infrastructure/JointData.h"

#ifndef WALKING_SIMULATOR
#include "Core/Math/Pose2D.h"
#else
#include "math/Pose2D.h"
#include "bhumanstub.h"
#endif

/**
 * @class KinematicOutput
 * A class that represents the output of the walking engine.
 */
STREAMABLE_ALIAS(KinematicOutput, JointRequest, {});

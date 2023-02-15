/**
 * @file ForwardKinematic.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A> and
 * <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
 */

#pragma once

#include "Tools/RobotParts/Arms.h"
#include "Tools/RobotParts/Limbs.h"

class JointData;
class Pose3D;
class RobotDimensions;

class ForwardKinematic {
public:
  static void calculateArmChain(bool left,
                                const JointData& joints,
                                const RobotDimensions& robotDimensions,
                                Pose3D limbs[Limbs::numOfLimbs]);
  static void calculateLegChain(bool left,
                                const JointData& joints,
                                const RobotDimensions& robotDimensions,
                                Pose3D limbs[Limbs::numOfLimbs]);
  static void
  calculateHeadChain(const JointData& joints, const RobotDimensions& robotDimensions, Pose3D limbs[Limbs::numOfLimbs]);
}; // namespace ForwardKinematic

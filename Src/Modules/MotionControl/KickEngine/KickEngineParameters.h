/**
 * @file KickEngineParameters.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#pragma once

#include <vector>

#include "Core/Math/Vector2.h"
#include "Core/Math/Vector3.h"
#include "Core/Enum.h"
#include "Core/Range.h"
#include "Representations/Infrastructure/JointData.h"

#define NUM_OF_POINTS 3

STREAMABLE(omniKickParameters,
           {
             ,
             (Vector2<>)supportfootCollisionOffset,
             (Vector2<>)maxFootReach,
             (Vector2<>)kickReferencePointOffset,
             (Vector2<>)kickDirectionRefPoint,
             (bool)useDuration,
             (float)previewHorizon,
             (float)timeStepDuration,
             (bool)usePreviewController,
             (bool)disableElbow,
             (float)kickHeight,
             (std::vector<float>)lqrGain,
             (std::vector<float>)previewControllerGain,
             (float)minimumDistance,
             (float)coMOffset,
             (bool)newBalanceMethod,
             (float)phaseDivider,
             (Rangef)strictFootRoll,
             (Rangef)speedModeRange,
             (Rangef)durationModeRange,
             (float)forwardTiltAngleConst,
             (float)sidewardTiltAngleConst,
             (float)maxTiltingSpeed,
             (float)upperBodySwingCompensationFactorForward,
             (float)upperBodySwingCompensationFactorSideward,
             (float)swingFootRotationMultiplier,
             (float)exitThreshHold,
           });

class Phase : public Streamable { // NOLINT: do not apply readability braces fix here
public:
  ENUM(Limb, leftFootTra, leftFootRot, rightFootTra, rightFootRot, leftArmTra, leftHandRot, rightArmTra, rightHandRot);

  enum { numOfPoints = 3 };

  unsigned int duration;

  Vector3<> controlPoints[Phase::numOfLimbs][numOfPoints];
  Vector2<> comTra[numOfPoints];
  Vector2<> headTra[numOfPoints];

  Vector3<> originPos[Phase::numOfLimbs];
  Vector2<> comOriginPos;
  Vector2<> comOriginOffset;
  Vector2<> headOrigin;
  Vector3<> odometryOffset;

protected:
  void serialize(In* in, Out* out) override;
};

STREAMABLE(BoostAngle, {
public:
  static const char *getName(JointData::Joint j) {
    return JointData::getName(j);
}
, (JointData::Joint)joint, (float)angle,
});

class KickEngineParameters : public Streamable {
public:
  int numberOfPhases = 0;
  char name[260];
  void calcControlPoints();

  Vector3<> getPosition(const float& phase, const int& phaseNumber, const int& limb);
  Vector3<> getPositionWithStraightLineExtension(const float& phase,
                                                 const int& phaseNumber,
                                                 const int& limb,
                                                 const omniKickParameters& omniKickParams);

  Vector2<> getComRefPosition(const float& phase, const int& phaseNumber);
  Vector2<> getHeadRefPosition(const float& phase, const int& phaseNumber);

  void initFirstPhase();
  void initFirstPhase(const Vector3<>* origins, const Vector2<>& head);
  void initFirstPhaseLoop(const Vector3<>* origins, const Vector2<>& lastCom, const Vector2<>& head);

  std::vector<BoostAngle> boostAngles; /**< Used joints for boosting. */

  /**< Reference values for the limbs. */
  Vector3<> footOrigin;
  Vector3<> footRotOrigin;
  Vector3<> armOrigin;
  Vector3<> handRotOrigin;
  Vector2<> comOrigin;
  Vector2<> headOrigin;

  /**< PID-Controller balance parameters. */
  float kpx;
  float kix;
  float kdx;
  float kpy;
  float kiy;
  float kdy;

  bool loop;      /**< Repeat the kick . */
  bool standLeft; /**< Is the left foot the support foot. */
  bool mirror;
  bool ignoreHead;            /**< Shall the head be ignored. */
  int adjustKickFootPosition; /**< The keyframe number of the current kick, at which the kicking foots z- and x-translation
                                  are adjusted. */

  std::vector<Phase> phaseParameters; /**< The keyframes for the kick. */
  Vector3<> phaseControlPointOffset;
  int kickSwingPhaseID;

private:
  void serialize(In* in, Out* out) override;
};

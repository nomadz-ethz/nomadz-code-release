/**
 * @file KickEngineParameters.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#include "KickEngineParameters.h"

void Phase::serialize(In* in, Out* out) {
  STREAM_REGISTER_BEGIN;

  STREAM(duration);

  Vector3<>& leftFootTra1(controlPoints[leftFootTra][1]);
  Vector3<>& leftFootTra2(controlPoints[leftFootTra][2]);
  Vector3<>& leftFootRot1(controlPoints[leftFootRot][1]);
  Vector3<>& leftFootRot2(controlPoints[leftFootRot][2]);
  Vector3<>& rightFootTra1(controlPoints[rightFootTra][1]);
  Vector3<>& rightFootTra2(controlPoints[rightFootTra][2]);
  Vector3<>& rightFootRot1(controlPoints[rightFootRot][1]);
  Vector3<>& rightFootRot2(controlPoints[rightFootRot][2]);
  Vector3<>& leftArmTra1(controlPoints[leftArmTra][1]);
  Vector3<>& leftArmTra2(controlPoints[leftArmTra][2]);
  Vector3<>& leftHandRot1(controlPoints[leftHandRot][1]);
  Vector3<>& leftHandRot2(controlPoints[leftHandRot][2]);
  Vector3<>& rightArmTra1(controlPoints[rightArmTra][1]);
  Vector3<>& rightArmTra2(controlPoints[rightArmTra][2]);
  Vector3<>& rightHandRot1(controlPoints[rightHandRot][1]);
  Vector3<>& rightHandRot2(controlPoints[rightHandRot][2]);
  Vector2<>& comTra1(comTra[1]);
  Vector2<>& comTra2(comTra[2]);
  Vector2<>& headTra1(headTra[1]);
  Vector2<>& headTra2(headTra[2]);

  STREAM(leftFootTra1)
  STREAM(leftFootTra2)
  STREAM(leftFootRot1)
  STREAM(leftFootRot2)
  STREAM(rightFootTra1)
  STREAM(rightFootTra2)
  STREAM(rightFootRot1)
  STREAM(rightFootRot2)
  STREAM(leftArmTra1)
  STREAM(leftArmTra2)
  STREAM(leftHandRot1)
  STREAM(leftHandRot2)
  STREAM(rightArmTra1)
  STREAM(rightArmTra2)
  STREAM(rightHandRot1)
  STREAM(rightHandRot2)
  STREAM(comTra1)
  STREAM(comTra2)
  STREAM(headTra1)
  STREAM(headTra2)

  STREAM(odometryOffset)

  STREAM_REGISTER_FINISH;
}

// void Phase::reg()
// {
//   PUBLISH(reg);
//   REG_CLASS(Phase);
//   REG(duration);
//   REG(Vector3<>, leftFootTra1);
//   REG(Vector3<>, leftFootTra2);
//   REG(Vector3<>, leftFootRot1);
//   REG(Vector3<>, leftFootRot2);
//   REG(Vector3<>, rightFootTra1);
//   REG(Vector3<>, rightFootTra2);
//   REG(Vector3<>, rightFootRot1);
//   REG(Vector3<>, rightFootRot2);
//   REG(Vector3<>, leftArmTra1);
//   REG(Vector3<>, leftArmTra2);
//   REG(Vector3<>, leftHandRot1);
//   REG(Vector3<>, leftHandRot2);
//   REG(Vector3<>, rightArmTra1);
//   REG(Vector3<>, rightArmTra2);
//   REG(Vector3<>, rightHandRot1);
//   REG(Vector3<>, rightHandRot2);
//   REG(Vector2<>, comTra1);
//   REG(Vector2<>, comTra2);
//   REG(Vector2<>, headTra1);
//   REG(Vector2<>, headTra2);
//   REG(odometryOffset);
// }

// void KickEngineParameters::onRead()
// {
//   numberOfPhases = static_cast<int>(phaseParameters.size());
//   calcControlPoints();
// }

void KickEngineParameters::serialize(In* in, Out* out) {
  STREAM_REGISTER_BEGIN;

  STREAM(footOrigin)
  STREAM(footRotOrigin)
  STREAM(armOrigin)
  STREAM(handRotOrigin)
  STREAM(comOrigin)
  STREAM(headOrigin)

  STREAM(kpx)
  STREAM(kix)
  STREAM(kdx)
  STREAM(kpy)
  STREAM(kiy)
  STREAM(kdy)

  STREAM(loop)
  STREAM(standLeft)
  STREAM(ignoreHead)
  STREAM(adjustKickFootPosition)
  STREAM(kickSwingPhaseID)

  if (in) {
    phaseParameters.clear();
  }
  STREAM(phaseParameters);
  numberOfPhases = phaseParameters.size();

  if (in) {
    calcControlPoints();
  }

  STREAM_REGISTER_FINISH;
}

void KickEngineParameters::calcControlPoints() {
  for (int phaseNumber = 0; phaseNumber < numberOfPhases - 1; phaseNumber++) {
    float factor = static_cast<float>(phaseParameters[phaseNumber].duration) /
                   static_cast<float>(phaseParameters[phaseNumber + 1].duration);

    phaseParameters[phaseNumber + 1].comTra[0] =
      phaseParameters[phaseNumber].comTra[2] - phaseParameters[phaseNumber].comTra[1];

    phaseParameters[phaseNumber + 1].comTra[0] *= factor;

    phaseParameters[phaseNumber + 1].comTra[0] += phaseParameters[phaseNumber].comTra[2];

    phaseParameters[phaseNumber + 1].headTra[0] =
      phaseParameters[phaseNumber].headTra[2] - phaseParameters[phaseNumber].headTra[1];

    phaseParameters[phaseNumber + 1].headTra[0] *= factor;

    phaseParameters[phaseNumber + 1].headTra[0] += phaseParameters[phaseNumber].headTra[2];

    for (int limb = 0; limb < Phase::numOfLimbs; limb++) {
      phaseParameters[phaseNumber + 1].controlPoints[limb][0] =
        phaseParameters[phaseNumber].controlPoints[limb][2] - phaseParameters[phaseNumber].controlPoints[limb][1];

      phaseParameters[phaseNumber + 1].controlPoints[limb][0] *= factor;

      phaseParameters[phaseNumber + 1].controlPoints[limb][0] += phaseParameters[phaseNumber].controlPoints[limb][2];
    }
  }
}

Vector3<> KickEngineParameters::getPosition(const float& phase, const int& phaseNumber, const int& limb) {
  Vector3<> p0, p1, p2;
  if (phaseNumber == 0) {
    p0 = phaseParameters[phaseNumber].originPos[limb];
  } else {
    Vector3<> phaseControlPointOffsetTarget =
      (phaseNumber == kickSwingPhaseID + 1 && limb == Phase::rightFootTra) ? phaseControlPointOffset : Vector3<>();
    p0 = phaseParameters[phaseNumber - 1].controlPoints[limb][2] + phaseControlPointOffsetTarget;
  }

  p1 = phaseParameters[phaseNumber].controlPoints[limb][1];
  p2 = phaseParameters[phaseNumber].controlPoints[limb][2];
  // float phaseSquared = 0.9 * phase * phase + 0.1 * phase;
  return (p0 - p1 * 2 + p2) * phase * phase + (-p0 * 2 + p1 * 2) * phase + p0;
}

Vector3<> KickEngineParameters::getPositionWithStraightLineExtension(const float& phase,
                                                                     const int& phaseNumber,
                                                                     const int& limb,
                                                                     const omniKickParameters& omniKickParams) {
  Vector3<> p0, p1, p2;
  p0 = phaseParameters[phaseNumber - 1].controlPoints[limb][2];
  p1 = phaseParameters[phaseNumber].controlPoints[limb][1];
  p2 = phaseParameters[phaseNumber].controlPoints[limb][2];
  float phaseAdjusted = phase / omniKickParams.phaseDivider;
  if (phase <= omniKickParams.phaseDivider) {
    return (p0 - p1 * 2 + p2) * phaseAdjusted * phaseAdjusted + (-p0 * 2 + p1 * 2) * phaseAdjusted + p0;
  } else {
    float a = -1 / (2 * (1 / omniKickParams.phaseDivider - 1));
    float b = 1;
    phaseControlPointOffset = (p2 - p1) * 2 *
                              (a * (1 / omniKickParams.phaseDivider - 1) * (1 / omniKickParams.phaseDivider - 1) +
                               b * (1 / omniKickParams.phaseDivider - 1));
    return (p2 - p1) * 2 * (a * (phaseAdjusted - 1) * (phaseAdjusted - 1) + b * (phaseAdjusted - 1)) + p2;
  }
}

Vector2<> KickEngineParameters::getComRefPosition(const float& phase, const int& phaseNumber) {
  Vector2<> p0, p1, p2;
  if (phaseNumber == 0) {
    p0 = phaseParameters[phaseNumber].comOriginPos;
  } else {
    p0 = phaseParameters[phaseNumber - 1].comTra[2];
  }

  p1 = phaseParameters[phaseNumber].comTra[1];
  p2 = phaseParameters[phaseNumber].comTra[2];

  // bezier
  return (p0 - p1 * 2 + p2) * phase * phase + (-p0 * 2 + p1 * 2) * phase + p0;
}

Vector2<> KickEngineParameters::getHeadRefPosition(const float& phase, const int& phaseNumber) {
  Vector2<> p0;
  if (phaseNumber == 0) {
    p0 = phaseParameters[phaseNumber].headOrigin;
  } else {
    p0 = phaseParameters[phaseNumber - 1].headTra[2];
  }

  const Vector2<> p1 = phaseParameters[phaseNumber].headTra[1];
  const Vector2<> p2 = phaseParameters[phaseNumber].headTra[2];

  return (p0 - p1 * 2 + p2) * phase * phase + (-p0 * 2 + p1 * 2) * phase + p0;
}

void KickEngineParameters::initFirstPhase() {
  // this function is only called by kickView
  if (numberOfPhases > 0) {
    phaseParameters[0].originPos[Phase::leftFootTra] = footOrigin;
    phaseParameters[0].originPos[Phase::rightFootTra] = Vector3<>(footOrigin.x, -footOrigin.y, footOrigin.z);

    phaseParameters[0].originPos[Phase::leftFootRot] = footRotOrigin;
    phaseParameters[0].originPos[Phase::rightFootRot] = Vector3<>(-footRotOrigin.x, footRotOrigin.y, -footRotOrigin.z);

    phaseParameters[0].originPos[Phase::leftArmTra] = armOrigin;
    phaseParameters[0].originPos[Phase::rightArmTra] = Vector3<>(armOrigin.x, -armOrigin.y, armOrigin.z);

    phaseParameters[0].originPos[Phase::leftHandRot] = handRotOrigin;
    phaseParameters[0].originPos[Phase::rightHandRot] = Vector3<>(-handRotOrigin.x, handRotOrigin.y, -handRotOrigin.z);

    // set the Offset for the first Phase to zero, because all calculations based on the startOrigin

    phaseParameters[0].comOriginPos = comOrigin;
    phaseParameters[0].comOriginOffset = Vector2<>();

    phaseParameters[0].headOrigin = headOrigin;

    phaseParameters[0].controlPoints[Phase::leftFootTra][0] = footOrigin;
    phaseParameters[0].controlPoints[Phase::rightFootTra][0] = Vector3<>(footOrigin.x, -footOrigin.y, footOrigin.z);

    phaseParameters[0].controlPoints[Phase::leftFootRot][0] = footRotOrigin;
    phaseParameters[0].controlPoints[Phase::rightFootRot][0] =
      Vector3<>(-footRotOrigin.x, footRotOrigin.y, -footRotOrigin.z);

    phaseParameters[0].controlPoints[Phase::leftArmTra][0] = armOrigin;
    phaseParameters[0].controlPoints[Phase::rightArmTra][0] = Vector3<>(armOrigin.x, -armOrigin.y, armOrigin.z);

    phaseParameters[0].controlPoints[Phase::leftHandRot][0] = handRotOrigin;
    phaseParameters[0].controlPoints[Phase::rightHandRot][0] =
      Vector3<>(-handRotOrigin.x, handRotOrigin.y, -handRotOrigin.z);

    phaseParameters[0].comTra[0] = comOrigin;
  }
}

void KickEngineParameters::initFirstPhase(const Vector3<>* origins, const Vector2<>& head) {
  for (int i = 0; i < Phase::numOfLimbs; ++i) {
    phaseParameters[0].originPos[i] = origins[i];
  }
  phaseParameters[0].comOriginPos = Vector2<>();
  phaseParameters[0].comOriginOffset = Vector2<>();
  phaseParameters[0].headOrigin = head;
}

void KickEngineParameters::initFirstPhaseLoop(const Vector3<>* origins, const Vector2<>& lastCom, const Vector2<>& head) {
  for (int i = 0; i < Phase::numOfLimbs; ++i) {
    phaseParameters[0].originPos[i] = origins[i];
  }
  phaseParameters[0].comOriginPos = lastCom;
  phaseParameters[0].comOriginOffset = Vector2<>();
  phaseParameters[0].headOrigin = head;
}

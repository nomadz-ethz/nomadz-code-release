/**
 * @file KickEngineData.cpp
 *
 * This file declares a module that creates the kicking motions.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#include <cstring>

#include "KickEngineData.h"
#include "Tools/RobotParts/Limbs.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Debugging/Modify.h"
#include "Core/Math/Pose3D.h"
#include "Core/Math/RotationMatrix.h"
#include "Core/Range.h"
#include "Core/System/SystemCall.h"
#include <numeric>

bool KickEngineData::getMotionIDByName(const KickRequest& kr, const std::vector<KickEngineParameters>& params) {
  motionID = -1;

  for (unsigned int i = 0; i < params.size(); ++i) {
    if (kr.getKickMotionFromName(&params[i].name[0]) == kr.kickMotionType) {
      motionID = i;
      return true;
    }
  }

  return false;
}

void KickEngineData::calculateOrigins(const KickRequest& kr,
                                      const JointData& ja,
                                      const TorsoMatrix& to,
                                      const RobotDimensions& robotDimensions) {
  if (!wasActive) {
    if (!kr.mirror) {
      origins[Phase::leftFootTra] = robotModel.limbs[Limbs::footLeft].rotation.invert() * robotModel.soleLeft.translation;
      origins[Phase::rightFootTra] = robotModel.limbs[Limbs::footRight].rotation.invert() * robotModel.soleRight.translation;
      origins[Phase::leftArmTra] = robotModel.limbs[Limbs::foreArmLeft].translation;
      origins[Phase::rightArmTra] = robotModel.limbs[Limbs::foreArmRight].translation;

      origins[Phase::leftFootRot] = Vector3<>(0, 0, robotModel.limbs[Limbs::footLeft].rotation.getZAngle());
      origins[Phase::rightFootRot] = Vector3<>(0, 0, robotModel.limbs[Limbs::footRight].rotation.getZAngle());
      origins[Phase::leftHandRot] = robotModel.limbs[Limbs::foreArmLeft].rotation.getAngleAxis();
      origins[Phase::rightHandRot] = robotModel.limbs[Limbs::foreArmRight].rotation.getAngleAxis();
    } else {
      origins[Phase::leftFootTra] = robotModel.limbs[Limbs::footRight].rotation.invert() * robotModel.soleRight.translation;
      origins[Phase::rightFootTra] = robotModel.limbs[Limbs::footLeft].rotation.invert() * robotModel.soleLeft.translation;
      origins[Phase::leftArmTra] = robotModel.limbs[Limbs::foreArmRight].translation;
      origins[Phase::rightArmTra] = robotModel.limbs[Limbs::foreArmLeft].translation;

      origins[Phase::leftFootRot] = Vector3<>(0, 0, ja.angles[JointData::RHipYawPitch]);
      origins[Phase::rightFootRot] = Vector3<>(0, 0, ja.angles[JointData::LHipYawPitch]);
      origins[Phase::leftHandRot] = -robotModel.limbs[Limbs::foreArmRight].rotation.getAngleAxis();
      origins[Phase::rightHandRot] = -robotModel.limbs[Limbs::foreArmLeft].rotation.getAngleAxis();

      origins[Phase::leftHandRot].y *= -1;
      origins[Phase::rightHandRot].y *= -1;

      origins[Phase::leftFootTra].y *= -1;
      origins[Phase::rightFootTra].y *= -1;
      origins[Phase::leftArmTra].y *= -1;
      origins[Phase::rightArmTra].y *= -1;
    }
  } else {
    for (int i = 0; i < Phase::numOfLimbs; ++i) {
      origins[i] = currentParameters.phaseParameters[currentParameters.numberOfPhases - 1].controlPoints[i][2];
    }
  }
}

void KickEngineData::calcPhaseState() {
  phase =
    static_cast<float>(timeSinceTimestamp) / static_cast<float>(currentParameters.phaseParameters[phaseNumber].duration);
}

bool KickEngineData::kickPhaseReached() {
  return phaseNumber >= currentParameters.kickSwingPhaseID;
}

bool KickEngineData::checkPhaseTime(const FrameInfo& frame, const JointData& ja, const TorsoMatrix& torsoMatrix) {
  timeSinceTimestamp = frame.getTimeSince(timestamp);
  timeSinceKickStarted = frame.getTimeSince(timestampKickStarted);
  if (motionID < 0) {
    return false;
  }

  // Is our current Keyframe valid?
  if (phaseNumber < currentParameters.numberOfPhases) {
    // Our current Keyframe is over
    if (static_cast<unsigned int>(timeSinceTimestamp) >= currentParameters.phaseParameters[phaseNumber].duration) {
      phaseNumber++;
      timestamp = frame.time;
      timeSinceTimestamp = frame.getTimeSince(timestamp);
      // Do we have a valid Keyframe left?
      if (phaseNumber < currentParameters.numberOfPhases) {
        if (currentKickRequest.armsBackFix) {
          std::cout << "FIX!" << std::endl;
          // Check which Arm shall be moved to the front
          if (lElbowFront) {
            Vector3<> invert = currentParameters.phaseParameters[phaseNumber].controlPoints[Phase::leftHandRot][2];
            invert.x *= -1.f;
            addDynPoint(DynPoint(Phase::leftHandRot, phaseNumber, invert), torsoMatrix);
          }
          if (rElbowFront) {
            Vector3<> invert = currentParameters.phaseParameters[phaseNumber].controlPoints[Phase::rightHandRot][2];
            invert.x *= -1.f;
            addDynPoint(DynPoint(Phase::rightHandRot, phaseNumber, invert), torsoMatrix);
          }
        }
        if (phaseNumber - 1 > 0 && fastKickEndAdjusted) {
          fastKickEndAdjusted = false;
          currentParameters.phaseParameters[phaseNumber - 1].controlPoints[Phase::rightFootTra][2].z = adjustedZValue;
          currentParameters.phaseParameters[phaseNumber - 1].controlPoints[Phase::rightFootTra][2].x = adjustedXValue;
        }

        // Calculate the controll points
        for (unsigned int i = 0; i < currentKickRequest.dynPoints.size(); i++) {
          if (currentKickRequest.dynPoints[i].phaseNumber == phaseNumber) {
            addDynPoint(currentKickRequest.dynPoints[i], torsoMatrix);
          }
        }
      }
    }
  } else if (currentParameters.loop && phaseNumber == currentParameters.numberOfPhases) {
    phaseNumber = 0;
    // calculateOrigins(currentKickRequest, ja, torsoMatrix);
    currentParameters.initFirstPhaseLoop(origins,
                                         currentParameters.phaseParameters[currentParameters.numberOfPhases - 1].comTra[2],
                                         Vector2<>(ja.angles[JointData::HeadPitch], ja.angles[JointData::HeadYaw]));

    for (unsigned int i = 0; i < currentKickRequest.dynPoints.size(); i++) {
      if (currentKickRequest.dynPoints[i].phaseNumber == phaseNumber) {
        addDynPoint(currentKickRequest.dynPoints[i], torsoMatrix);
      }
    }
  }

  return phaseNumber < currentParameters.numberOfPhases;
}

bool KickEngineData::calcJoints(JointRequest& jointRequest,
                                const RobotDimensions& rd,
                                const DamageConfiguration& theDamageConfiguration) {
  // Calculate Legs
  if (motionID > -1) {
    if (!currentParameters.ignoreHead) {
      jointRequest.jointAngles.angles[JointData::HeadPitch] = head.x;
      jointRequest.jointAngles.angles[JointData::HeadYaw] = head.y;
    } else {
      jointRequest.jointAngles.angles[JointData::HeadYaw] = JointData::ignore;
      jointRequest.jointAngles.angles[JointData::HeadPitch] = JointData::ignore;
    }

    calcLegJoints(JointData::LHipYawPitch, jointRequest, rd, theDamageConfiguration);
    calcLegJoints(JointData::RHipYawPitch, jointRequest, rd, theDamageConfiguration);

    simpleCalcArmJoints(
      JointData::LShoulderPitch, jointRequest, rd, positions[Phase::leftArmTra], positions[Phase::leftHandRot]);
    simpleCalcArmJoints(
      JointData::RShoulderPitch, jointRequest, rd, positions[Phase::rightArmTra], positions[Phase::rightHandRot]);

    return true;
  } else // just set the angles from init
  {
    for (int i = JointData::LShoulderPitch; i < JointData::numOfJoints; ++i) {
      jointRequest.jointAngles.angles[i] = lastBalancedJointRequest.jointAngles.angles[i];
    }

    return false;
  }
}

void KickEngineData::calcOdometryOffset(KickEngineOutput& output, const RobotModel& theRobotModel) {
  Pose3D ankleInAnkle;
  // quickhack to compute support foot, could be worng if the motion is using the right foot as support foot as default
  // could use theTorsoMatrix.leftSupportFoot, but if the swing foot steps into the ground the odometry jumps
  if (currentKickRequest.mirror) {
    ankleInAnkle = theRobotModel.limbs[Limbs::ankleRight].invert() * theRobotModel.limbs[Limbs::ankleLeft];
  } else {
    ankleInAnkle = theRobotModel.limbs[Limbs::ankleLeft].invert() * theRobotModel.limbs[Limbs::ankleRight];
  }

  Pose2D currentOdometry(ankleInAnkle.translation.x * 0.5f, ankleInAnkle.translation.y * 0.5f);

  output.odometryOffset = currentOdometry - lastOdometry;
  if (phase == 0) {
    output.odometryOffset += Pose2D(currentParameters.phaseParameters[phaseNumber].odometryOffset.x,
                                    currentParameters.phaseParameters[phaseNumber].odometryOffset.y);
  }

  lastOdometry = currentOdometry;
}

void KickEngineData::calcLegJoints(const JointData::Joint& joint,
                                   JointRequest& jointRequest,
                                   const RobotDimensions& theRobotDimensions,
                                   const DamageConfiguration& theDamageConfiguration) {
  const int sign = joint == JointData::LHipYawPitch ? 1 : -1;

  const Vector3<>& footPos = (sign > 0) ? positions[Phase::leftFootTra] : positions[Phase::rightFootTra];
  const Vector3<>& footRotAng = (sign > 0) ? positions[Phase::leftFootRot] : positions[Phase::rightFootRot];

  const RotationMatrix rotateBodyTilt = RotationMatrix::fromRotationX(bodyAngle.x).rotateY(bodyAngle.y);
  Vector3<> anklePos = rotateBodyTilt * (footPos - Vector3<>(0.f, 0, -theRobotDimensions.heightLeg5Joint));
  anklePos -= Vector3<>(0.f, sign * (theRobotDimensions.yHipOffset), 0);

  // const RotationMatrix zRot = RotationMatrix::aroundZ(footRotAng.z).rotateX(sign * pi_4);
  // anklePos = zRot * anklePos;
  const float leg0 = 0; // std::atan2(-anklePos.x, anklePos.y);
  const float diagonal = anklePos.abs();

  // upperLegLength, lowerLegLength, and diagonal form a triangle, use cosine theorem
  float a1 = (theRobotDimensions.upperLegLength * theRobotDimensions.upperLegLength -
              theRobotDimensions.lowerLegLength * theRobotDimensions.lowerLegLength + diagonal * diagonal) /
             (2 * theRobotDimensions.upperLegLength * diagonal);
  // if(std::abs(a1) > 1.f) OUTPUT_TEXT("clipped a1");
  a1 = std::abs(a1) > 1.f ? 0.f : std::acos(a1);

  float a2 = (theRobotDimensions.upperLegLength * theRobotDimensions.upperLegLength +
              theRobotDimensions.lowerLegLength * theRobotDimensions.lowerLegLength - diagonal * diagonal) /
             (2 * theRobotDimensions.upperLegLength * theRobotDimensions.lowerLegLength);
  // if(std::abs(a2) > 1.f) OUTPUT_TEXT("clipped a2");
  a2 = std::abs(a2) > 1.f ? pi : std::acos(a2);

  const float leg2 = -a1 - std::atan2(anklePos.x, Vector2<>(anklePos.y, anklePos.z).abs() * -sgn(anklePos.z));
  const float leg1 = anklePos.z == 0.0f ? 0.0f : (std::atan(anklePos.y / -anklePos.z));
  const float leg3 = pi - a2;

  const RotationMatrix rotateBecauseOfHip = RotationMatrix::fromRotationZ(0).rotateX(bodyAngle.x).rotateY(bodyAngle.y);
  // calculate invert foot rotation so that they are flat to the ground
  RotationMatrix footRot = RotationMatrix::fromRotationX(leg1).rotateY(leg2 + leg3);
  footRot = footRot.invert() /* * zRot*/ * rotateBecauseOfHip;

  // and add additonal foot rotation (which is probably not flat to the ground)
  const float leg4 = std::atan2(footRot[2][0], footRot[2][2]) + footRotAng.y;
  const float leg5 = std::asin(-footRot[2][1]) + footRotAng.x;

  jointRequest.jointAngles.angles[joint] = leg0;
  jointRequest.jointAngles.angles[joint + 1] = (/*-pi_4 * sign + */ leg1);
  jointRequest.jointAngles.angles[joint + 2] = leg2;
  jointRequest.jointAngles.angles[joint + 3] = leg3;
  jointRequest.jointAngles.angles[joint + 4] = leg4;
  jointRequest.jointAngles.angles[joint + 5] = omniKickParams.strictFootRoll.limit(leg5);

  // TODO hack which allows calibration, but works only if the left foot is the support foot in .kmc file
  // Vector2<> tiltCalibration = currentKickRequest.mirror ? theDamageConfiguration.startTiltRight :
  // theDamageConfiguration.startTiltLeft;
  // if(currentKickRequest.mirror)
  //   tiltCalibration.x *= -1.f;
  // jointRequest.jointAngles.angles[JointData::LAnkleRoll] += tiltCalibration.x;
  // jointRequest.jointAngles.angles[JointData::LAnklePitch] += tiltCalibration.y;
}

void KickEngineData::simpleCalcArmJoints(const JointData::Joint& joint,
                                         JointRequest& jointRequest,
                                         const RobotDimensions& theRobotDimensions,
                                         const Vector3<>& armPos,
                                         const Vector3<>& handRotAng) {
  float sign = joint == JointData::LShoulderPitch ? 1.f : -1.f;

  const Vector3<> target =
    armPos -
    Vector3<>(theRobotDimensions.armOffset.x, theRobotDimensions.armOffset.y * sign, theRobotDimensions.armOffset.z);

  jointRequest.jointAngles.angles[joint + 0] = std::atan2(target.z, target.x);
  jointRequest.jointAngles.angles[joint + 1] = std::atan2(target.y * sign, std::sqrt(sqr(target.x) + sqr(target.z)));

  float length2ElbowJoint = Vector3<>(theRobotDimensions.upperArmLength, theRobotDimensions.yElbowShoulder, 0.f).abs();
  float angle = std::asin(theRobotDimensions.upperArmLength / length2ElbowJoint);

  Pose3D elbow;
  elbow.rotateY(-jointRequest.jointAngles.angles[joint + 0])
    .rotateZ(jointRequest.jointAngles.angles[joint + 1])
    .translate(length2ElbowJoint, 0.f, 0.f)
    .rotateZ(-angle)
    .translate(theRobotDimensions.yElbowShoulder, 0.f, 0.f);

  jointRequest.jointAngles.angles[joint + 0] = std::atan2(elbow.translation.z, elbow.translation.x);
  jointRequest.jointAngles.angles[joint + 1] =
    std::atan2(elbow.translation.y, std::sqrt(sqr(elbow.translation.x) + sqr(elbow.translation.z)));
  jointRequest.jointAngles.angles[joint + 0] =
    (jointRequest.jointAngles.angles[joint + 0] < pi) ? jointRequest.jointAngles.angles[joint + 0] : 0; // clip
                                                                                                        // special

  Pose3D hand(elbow.translation);

  hand.rotateZ(handRotAng.z * sign)
    .rotateY(handRotAng.y)
    .rotateX(handRotAng.x * sign)
    .translate(theRobotDimensions.lowerArmLength, 0.f, 0.f);

  // calculate desired elbow "klapp"-angle
  const float c = hand.translation.abs(), a = theRobotDimensions.upperArmLength, b = theRobotDimensions.lowerArmLength;

  // cosine theorem
  float cosAngle = (-sqr(c) + sqr(b) + sqr(a)) / (2.0f * a * b);
  if (cosAngle < -1.0f) {
    cosAngle = -1.0f;
  } else if (cosAngle > 1.0f) {
    cosAngle = 1.0f;
  }
  jointRequest.jointAngles.angles[joint + 3] = std::acos(cosAngle) - pi;

  // calculate hand in elbow coordinate system and calculate last angle
  Pose3D shoulder2Elbow;
  shoulder2Elbow.translate(0, -theRobotDimensions.upperArmLength, 0);
  shoulder2Elbow.rotateX(-(jointRequest.jointAngles.angles[joint + 1] - pi / 2.0f));
  shoulder2Elbow.rotateY(jointRequest.jointAngles.angles[joint + 0] + pi / 2.0f);
  const Vector3<> handInEllbow = shoulder2Elbow * hand.translation;

  jointRequest.jointAngles.angles[joint + 2] = -(std::atan2(handInEllbow.z, handInEllbow.x) + pi / 2.f);
  while (jointRequest.jointAngles.angles[joint + 2] > pi) {
    jointRequest.jointAngles.angles[joint + 2] -= 2 * pi;
  }
  while (jointRequest.jointAngles.angles[joint + 2] < -pi) {
    jointRequest.jointAngles.angles[joint + 2] += 2 * pi;
  }

  jointRequest.jointAngles.angles[joint + 4] = -pi / 2 * sign; // BUG? I don't know if multiply by sign is correct here
  jointRequest.jointAngles.angles[joint + 5] = 0.f;

  jointRequest.jointAngles.angles[joint + 0] *= -1.f;
  jointRequest.jointAngles.angles[joint + 1] *= sign;
  jointRequest.jointAngles.angles[joint + 2] *= sign;
  jointRequest.jointAngles.angles[joint + 3] *= sign;

  if (omniKickParams.disableElbow) {
    jointRequest.jointAngles.angles[joint + 2] = 0;
    jointRequest.jointAngles.angles[joint + 3] = 0;
    jointRequest.jointAngles.angles[joint + 4] = 0;
  }
  // jointRequest.jointAngles.angles[joint + 4] = handRotAng.z;
  // jointRequest.jointAngles.angles[joint + 5] = 0.f;
}

float KickEngineData::linearInterpolation(const float& value, const float& goal, const float& maxSpeed) {
  if (std::abs(goal - value) > maxSpeed) {
    return value + (goal - value) / std::abs(goal - value) * maxSpeed;
  } else {
    return goal;
  }
}

void KickEngineData::balanceCOM(JointRequest& joints, const RobotDimensions& rd, const MassCalibration& mc) {
  if (omniKickParams.newBalanceMethod || (currentKickRequest.kickMotionType == KickRequest::omniKickDefault ||
                                          currentKickRequest.kickMotionType == KickRequest::omniKickLong)) {
    actualDiff = Vector3<>();
    Vector3<> comOffset;
    int mirrorFactor = 1;
    if (currentParameters.mirror) {
      mirrorFactor = -1;
    }
    int balanceAheadPhasePrevious = (currentParameters.kickSwingPhaseID == 3) ? 2 : 1;
    if (Rangei(currentParameters.kickSwingPhaseID - balanceAheadPhasePrevious, currentParameters.kickSwingPhaseID + 1)
          .isInside(phaseNumber)) {
      float leftFootTiltForward = std::atan2(positions[Phase::leftFootTra].x, -positions[Phase::leftFootTra].z);
      float rightFootTiltForward = std::atan2(positions[Phase::rightFootTra].x, -positions[Phase::rightFootTra].z);
      float balanceAngleYDynamicTarget =
        (leftFootTiltForward - rightFootTiltForward) * omniKickParams.upperBodySwingCompensationFactorForward;
      balanceAngleY = linearInterpolation(
        balanceAngleY, omniKickParams.forwardTiltAngleConst + balanceAngleYDynamicTarget, omniKickParams.maxTiltingSpeed);
      float leftFootTiltSide = std::atan2(positions[Phase::leftFootTra].y, -positions[Phase::leftFootTra].z);
      float rightFootTiltSide = std::atan2(positions[Phase::rightFootTra].y, -positions[Phase::rightFootTra].z);
      float balanceAngleXDynamicTarget =
        (leftFootTiltSide + rightFootTiltSide) * omniKickParams.upperBodySwingCompensationFactorSideward;
      balanceAngleX = linearInterpolation(
        balanceAngleX, balanceAngleXDynamicTarget + omniKickParams.sidewardTiltAngleConst, omniKickParams.maxTiltingSpeed);

      comOffset =
        Vector3<>(currentParameters.comOrigin.x, currentParameters.comOrigin.y + omniKickParams.coMOffset * mirrorFactor, 0);

      const Vector3<> com =
        RotationMatrix(mirrorFactor * (balanceAngleX), (balanceAngleY), 0) * robotModel.centerOfMass + comOffset;

      ref = RotationMatrix(mirrorFactor * (balanceAngleX), (balanceAngleY), 0) *
            ((currentParameters.mirror) ? robotModel.soleRight.translation : robotModel.soleLeft.translation);
      actualDiff = (com - ref);
      actualDiff.y *= mirrorFactor;
      PLOT("module:KickEngine:actualDiffY", actualDiff.y);
      PLOT("module:KickEngine:comY", com.y);
      PLOT("module:KickEngine:refY", ref.y);

    } else {
      balanceAngleY = linearInterpolation(balanceAngleY, 0, omniKickParams.maxTiltingSpeed);
      actualDiff = Vector3<>();
      balanceAngleX = linearInterpolation(balanceAngleX, 0, omniKickParams.maxTiltingSpeed);
    }
    if ((balanceAdjustment - actualDiff.toVec2()).abs() > 0.5) {
      balanceAdjustment += (actualDiff.toVec2() - balanceAdjustment).normalize() * 0.5;
    } else {
      balanceAdjustment += actualDiff.toVec2() - balanceAdjustment;
    }
    if (Rangei(currentParameters.kickSwingPhaseID - balanceAheadPhasePrevious, currentParameters.kickSwingPhaseID + 1)
          .isInside(phaseNumber)) {
      balanceSum += balanceAdjustment * cycletime;
    } else {
      balanceSum.x = linearInterpolation(balanceSum.x, 0, 1);
      balanceSum.y = linearInterpolation(balanceSum.y, 0, 1);
    }

    Vector2<> balance(currentParameters.kpx * (balanceAdjustment.x) + currentParameters.kix * balanceSum.x +
                        currentParameters.kdx * ((balanceAdjustment.x - lastCom.x) / cycletime),
                      currentParameters.kpy * (balanceAdjustment.y) + currentParameters.kiy * balanceSum.y +
                        currentParameters.kdy * ((balanceAdjustment.y - lastCom.y) / cycletime));
    lastCom = balanceAdjustment;
    PLOT("module:KickEngine:balanceY", balance.y);
    positions[Phase::leftFootTra] += balance;
    PLOT("module:KickEngine:leftreq", mirrorFactor * positions[Phase::leftFootTra].y);
    PLOT("module:KickEngine:rightreq", mirrorFactor * positions[Phase::rightFootTra].y);

    positions[Phase::rightFootTra] += balance;
    positions[Phase::leftFootTra] = RotationMatrix(-balanceAngleX, -balanceAngleY, 0) * positions[Phase::leftFootTra];
    positions[Phase::rightFootTra] =
      RotationMatrix(omniKickParams.swingFootRotationMultiplier * (-balanceAngleX), -balanceAngleY, 0) *
      positions[Phase::rightFootTra];
    positions[Phase::leftFootRot] += Vector3<>(-balanceAngleX, -balanceAngleY, 0);
    positions[Phase::rightFootRot] += Vector3<>(-balanceAngleX, -balanceAngleY, 0);

  } else {
    const Pose3D& torso = toLeftSupport ? comRobotModel.limbs[Limbs::footLeft] : comRobotModel.limbs[Limbs::footRight];
    comRobotModel.setJointData(joints.jointAngles, rd, mc);
    const Vector3<> com = torso.rotation.invert() * comRobotModel.centerOfMass;

    actualDiff = com - ref;

    balanceSum += Vector2<>(actualDiff.x, actualDiff.y);

    float height = comRobotModel.centerOfMass.z - ref.z;

    const Vector2<> balance(currentParameters.kpy * (actualDiff.x) + currentParameters.kiy * balanceSum.x +
                              currentParameters.kdy * ((actualDiff.x - lastCom.x) / cycletime),
                            -currentParameters.kpx * (actualDiff.y) + -currentParameters.kix * balanceSum.y +
                              -currentParameters.kdx * ((actualDiff.y - lastCom.y) / cycletime));

    if (height != 0.f) {
      bodyAngle.x = (balance.y != 0) ? std::atan2((balance.y), height) : 0;
      bodyAngle.y = (balance.x != 0) ? std::atan2((balance.x), height) : 0;
    }

    lastCom = actualDiff;
  }
}

void KickEngineData::mirrorIfNecessary(JointRequest& joints) {
  if (currentKickRequest.mirror) {
    JointRequest old = joints;
    for (int i = 0; i < JointData::numOfJoints; ++i) {
      if (i == JointData::HeadPitch) {
        continue;
      }

      joints.jointAngles.angles[i] = old.mirror(static_cast<JointData::Joint>(i));
    }
  }
}
void KickEngineData::BOOST(JointRequest& jointRequest, int boostPhase) {
  if (currentKickRequest.mirror) {
    JointRequest modifiedRequest;
    modifiedRequest.mirror(jointRequest);
    for (const BoostAngle& boostAngle : currentParameters.boostAngles) {
      if ((boostAngle.joint != JointData::LAnklePitch && boostAngle.joint != JointData::RAnklePitch && boostPhase == 4) ||
          boostAngle.joint == JointData::LAnklePitch || boostAngle.joint == JointData::RAnklePitch) {
        modifiedRequest.jointAngles.angles[boostAngle.joint] = boostAngle.angle;
      }
    }
    jointRequest.mirror(modifiedRequest);
  } else {
    for (const BoostAngle& boostAngle : currentParameters.boostAngles) {
      if ((boostAngle.joint != JointData::LAnklePitch && boostAngle.joint != JointData::RAnklePitch && boostPhase == 4) ||
          boostAngle.joint == JointData::LAnklePitch || boostAngle.joint == JointData::RAnklePitch) {
        jointRequest.jointAngles.angles[boostAngle.joint] = boostAngle.angle;
      }
    }
  }
}

void KickEngineData::addGyroBalance(JointRequest& jointRequest,
                                    const JointCalibration& jointCalibration,
                                    const InertiaSensorData& id,
                                    const float& ratio) {
  if (id.gyro.y != 0 && id.gyro.x != 0 && !willBeLeft) {
    // Predict next gyrodata
    gyro = Vector2<>(id.gyro.x, id.gyro.y) * 0.3f + gyro * 0.7f;

    // some clipping
    for (int i = JointData::LHipYawPitch; i < JointData::numOfJoints; i++) {
      Range<>(jointCalibration.joints[i].minAngle, jointCalibration.joints[i].maxAngle)
        .limit(jointRequest.jointAngles.angles[i]);
    }

    const JointRequest balancedJointRequest = jointRequest;

    // calculate the commandedVelocity
    float commandedVelocity[4];
    // y-velocity if left leg is support
    commandedVelocity[0] = (balancedJointRequest.jointAngles.angles[JointData::LHipPitch] -
                            lastBalancedJointRequest.jointAngles.angles[JointData::LHipPitch]) /
                           cycletime;
    // y-velocity if right leg is support
    commandedVelocity[1] = (balancedJointRequest.jointAngles.angles[JointData::RHipPitch] -
                            lastBalancedJointRequest.jointAngles.angles[JointData::RHipPitch]) /
                           cycletime;
    // x-velcocity if left leg is support
    commandedVelocity[2] = (balancedJointRequest.jointAngles.angles[JointData::LHipRoll] -
                            lastBalancedJointRequest.jointAngles.angles[JointData::LHipRoll]) /
                           cycletime;
    // x-velocity if right leg is support
    commandedVelocity[3] = (balancedJointRequest.jointAngles.angles[JointData::RHipRoll] -
                            lastBalancedJointRequest.jointAngles.angles[JointData::RHipRoll]) /
                           cycletime;

    // calculate disturbance from meseaured velocity and commanded velocity
    // y-velocity if left leg is support
    float gyroVelyLeft = (gyro.y + commandedVelocity[0] - lastGyroLeft.y) / cycletime;
    lastGyroLeft.y = gyro.y + commandedVelocity[0];
    // y-velocity if right leg is support
    float gyroVelyRight = (gyro.y + commandedVelocity[1] - lastGyroRight.y) / cycletime;
    lastGyroRight.y = gyro.y + commandedVelocity[1];
    // x-velocity if left leg is support
    float gyroVelxLeft = (gyro.x + commandedVelocity[2] - lastGyroLeft.x) / cycletime;
    lastGyroLeft.x = gyro.x + commandedVelocity[2];
    // x-velocity if right leg is support
    float gyroVelxRight = (gyro.x + commandedVelocity[3] - lastGyroRight.x) / cycletime;
    lastGyroRight.x = gyro.x + commandedVelocity[3];

    // calculate control variable with PID-Control
    float calcVelocity[4];
    // y if left supprt
    calcVelocity[0] = -gyroP.y * (gyro.y + commandedVelocity[0]) - gyroD.y * gyroVelyLeft - gyroI.y * (gyroErrorLeft.y);
    // y if right support
    calcVelocity[1] = -gyroP.y * (gyro.y + commandedVelocity[1]) - gyroD.y * gyroVelyRight - gyroI.y * (gyroErrorRight.y);
    // x if left support
    calcVelocity[2] = -gyroP.x * (gyro.x + commandedVelocity[2]) + gyroD.x * gyroVelxLeft + gyroI.x * gyroErrorLeft.x;
    // x if right support
    calcVelocity[3] = -gyroP.x * (gyro.x - commandedVelocity[3]) + gyroD.x * gyroVelxRight + gyroI.x * gyroErrorRight.x;

    bool supp = (currentKickRequest.mirror) ? !toLeftSupport : toLeftSupport;

    if (supp) // last support Leg was left
    {
      // y
      jointRequest.jointAngles.angles[JointData::RHipPitch] += calcVelocity[0] * cycletime * ratio;
      jointRequest.jointAngles.angles[JointData::RHipPitch] += calcVelocity[0] * cycletime * ratio;
      jointRequest.jointAngles.angles[JointData::LAnklePitch] += calcVelocity[0] * cycletime * ratio;
      jointRequest.jointAngles.angles[JointData::RAnklePitch] += calcVelocity[0] * cycletime * ratio;
      // x
      jointRequest.jointAngles.angles[JointData::LHipRoll] += calcVelocity[2] * cycletime * ratio;
      jointRequest.jointAngles.angles[JointData::RHipRoll] += calcVelocity[2] * cycletime * ratio;
      jointRequest.jointAngles.angles[JointData::LAnkleRoll] -= calcVelocity[2] * cycletime * ratio;
      jointRequest.jointAngles.angles[JointData::RAnkleRoll] -= calcVelocity[2] * cycletime * ratio;
    } else // if(toRightSupport)
    {
      // y
      jointRequest.jointAngles.angles[JointData::RHipPitch] += calcVelocity[1] * cycletime * ratio;
      jointRequest.jointAngles.angles[JointData::LHipPitch] += calcVelocity[1] * cycletime * ratio;
      jointRequest.jointAngles.angles[JointData::LAnklePitch] += calcVelocity[1] * cycletime * ratio;
      jointRequest.jointAngles.angles[JointData::RAnklePitch] += calcVelocity[1] * cycletime * ratio;

      // x
      jointRequest.jointAngles.angles[JointData::LHipRoll] += calcVelocity[3] * cycletime * ratio;
      jointRequest.jointAngles.angles[JointData::RHipRoll] += calcVelocity[3] * cycletime * ratio;
      jointRequest.jointAngles.angles[JointData::LAnkleRoll] -= calcVelocity[3] * cycletime * ratio;
      jointRequest.jointAngles.angles[JointData::RAnkleRoll] -= calcVelocity[3] * cycletime * ratio;
    }
    gyroErrorLeft += lastGyroLeft;
    gyroErrorRight += lastGyroRight;
    lastBalancedJointRequest = balancedJointRequest;
  }
}

bool KickEngineData::adjustFastKickHack(const TorsoMatrix& torsoMatrix) {
  // Adjust kicking foot position, to prevent that the foot pushes into the ground and results in a fall
  if (phaseNumber == currentParameters.adjustKickFootPosition) {
    // Foot position is already frozen
    if (fastKickEndAdjusted) {
      return true;
    }
    Pose3D foot1 = Pose3D();
    Pose3D foot2 = Pose3D();
    if (currentKickRequest.mirror) {
      foot1 = torsoMatrix.rotation * robotModel.soleLeft;
      foot2 = torsoMatrix.rotation * robotModel.soleRight;
    } else {
      foot1 = torsoMatrix.rotation * robotModel.soleRight;
      foot2 = torsoMatrix.rotation * robotModel.soleLeft;
    }
    float currentZDif = foot1.translation.z - lastZDif;
    // only z check is necessary
    // calculate kick foot height 3 frames into the future
    if (lastZDif != 0.f && currentZDif * 3.f + foot1.translation.z < foot2.translation.z) {
      return true;
    }
    lastZDif = foot1.translation.z;
    lastXDif = foot1.translation.x;
  }
  return false;
}

void KickEngineData::addDynPoint(const DynPoint& dynPoint, const TorsoMatrix& torsoMatrix) {
  Vector3<> d = dynPoint.translation;

  const int phaseNumber = dynPoint.phaseNumber;
  const int limb = dynPoint.limb;

  if (dynPoint.duration > 0) {
    currentParameters.phaseParameters[phaseNumber].duration = dynPoint.duration;
  }

  currentParameters.phaseParameters[phaseNumber].odometryOffset = dynPoint.odometryOffset;

  const Vector3<>& cubePoint = currentParameters.phaseParameters[phaseNumber].controlPoints[limb][2];
  const Vector3<> diff = cubePoint - d;

  currentParameters.phaseParameters[phaseNumber].controlPoints[limb][2] -= diff;
  currentParameters.phaseParameters[phaseNumber].controlPoints[limb][1] -= diff;

  if (phaseNumber < currentParameters.numberOfPhases - 1) {
    currentParameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] =
      currentParameters.phaseParameters[phaseNumber].controlPoints[limb][2] -
      currentParameters.phaseParameters[phaseNumber].controlPoints[limb][1];

    float factor = static_cast<float>(currentParameters.phaseParameters[phaseNumber + 1].duration) /
                   static_cast<float>(currentParameters.phaseParameters[phaseNumber].duration);
    currentParameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] *= factor;

    currentParameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] +=
      currentParameters.phaseParameters[phaseNumber].controlPoints[limb][2];
  }
}

void KickEngineData::addJointDataDebuggingPlot(const JointData& JD, const JointRequest& JDReq) {
  PLOT("module:KickEngine:JointData0", JD.angles[0] * 57.3);
  PLOT("module:KickEngine:JointData1", JD.angles[1] * 57.3);
  PLOT("module:KickEngine:JointData2", JD.angles[2] * 57.3);
  PLOT("module:KickEngine:JointData3", JD.angles[3] * 57.3);
  PLOT("module:KickEngine:JointData4", JD.angles[4] * 57.3);
  PLOT("module:KickEngine:JointData5", JD.angles[5] * 57.3);
  PLOT("module:KickEngine:JointData6", JD.angles[6] * 57.3);
  PLOT("module:KickEngine:JointData7", JD.angles[7] * 57.3);
  PLOT("module:KickEngine:JointData8", JD.angles[8] * 57.3);
  PLOT("module:KickEngine:JointData9", JD.angles[9] * 57.3);
  PLOT("module:KickEngine:JointData10", JD.angles[10] * 57.3);
  PLOT("module:KickEngine:JointData11", JD.angles[11] * 57.3);
  PLOT("module:KickEngine:JointData12", JD.angles[12] * 57.3);
  PLOT("module:KickEngine:JointData13", JD.angles[13] * 57.3);
  PLOT("module:KickEngine:JointData14", JD.angles[14] * 57.3);
  PLOT("module:KickEngine:JointData15", JD.angles[15] * 57.3);
  PLOT("module:KickEngine:JointData16", JD.angles[16] * 57.3);
  PLOT("module:KickEngine:JointData17", JD.angles[17] * 57.3);
  PLOT("module:KickEngine:JointData18", JD.angles[18] * 57.3);
  PLOT("module:KickEngine:JointData19", JD.angles[19] * 57.3);
  PLOT("module:KickEngine:JointData20", JD.angles[20] * 57.3);
  PLOT("module:KickEngine:JointData21", JD.angles[21] * 57.3);
  PLOT("module:KickEngine:JointData22", JD.angles[22] * 57.3);
  PLOT("module:KickEngine:JointData23", JD.angles[23] * 57.3);
  PLOT("module:KickEngine:JointData24", JD.angles[24] * 57.3);

  PLOT("module:KickEngine:JointDataReq0", JDReq.jointAngles.angles[0] * 57.3);
  PLOT("module:KickEngine:JointDataReq1", JDReq.jointAngles.angles[1] * 57.3);
  PLOT("module:KickEngine:JointDataReq2", JDReq.jointAngles.angles[2] * 57.3);
  PLOT("module:KickEngine:JointDataReq3", JDReq.jointAngles.angles[3] * 57.3);
  PLOT("module:KickEngine:JointDataReq4", JDReq.jointAngles.angles[4] * 57.3);
  PLOT("module:KickEngine:JointDataReq5", JDReq.jointAngles.angles[5] * 57.3);
  PLOT("module:KickEngine:JointDataReq6", JDReq.jointAngles.angles[6] * 57.3);
  PLOT("module:KickEngine:JointDataReq7", JDReq.jointAngles.angles[7] * 57.3);
  PLOT("module:KickEngine:JointDataReq8", JDReq.jointAngles.angles[8] * 57.3);
  PLOT("module:KickEngine:JointDataReq9", JDReq.jointAngles.angles[9] * 57.3);
  PLOT("module:KickEngine:JointDataReq10", JDReq.jointAngles.angles[10] * 57.3);
  PLOT("module:KickEngine:JointDataReq11", JDReq.jointAngles.angles[11] * 57.3);
  PLOT("module:KickEngine:JointDataReq12", JDReq.jointAngles.angles[12] * 57.3);
  PLOT("module:KickEngine:JointDataReq13", JDReq.jointAngles.angles[13] * 57.3);
  PLOT("module:KickEngine:JointDataReq14", JDReq.jointAngles.angles[14] * 57.3);
  PLOT("module:KickEngine:JointDataReq15", JDReq.jointAngles.angles[15] * 57.3);
  PLOT("module:KickEngine:JointDataReq16", JDReq.jointAngles.angles[16] * 57.3);
  PLOT("module:KickEngine:JointDataReq17", JDReq.jointAngles.angles[17] * 57.3);
  PLOT("module:KickEngine:JointDataReq18", JDReq.jointAngles.angles[18] * 57.3);
  PLOT("module:KickEngine:JointDataReq19", JDReq.jointAngles.angles[19] * 57.3);
  PLOT("module:KickEngine:JointDataReq20", JDReq.jointAngles.angles[20] * 57.3);
  PLOT("module:KickEngine:JointDataReq21", JDReq.jointAngles.angles[21] * 57.3);
  PLOT("module:KickEngine:JointDataReq22", JDReq.jointAngles.angles[22] * 57.3);
  PLOT("module:KickEngine:JointDataReq23", JDReq.jointAngles.angles[23] * 57.3);
  PLOT("module:KickEngine:JointDataReq24", JDReq.jointAngles.angles[24] * 57.3);

  PLOT("module:KickEngine:JointDataReqHard0", JDReq.jointHardness.hardness[0]);
  PLOT("module:KickEngine:JointDataReqHard1", JDReq.jointHardness.hardness[1]);
  PLOT("module:KickEngine:JointDataReqHard2", JDReq.jointHardness.hardness[2]);
  PLOT("module:KickEngine:JointDataReqHard3", JDReq.jointHardness.hardness[3]);
  PLOT("module:KickEngine:JointDataReqHard4", JDReq.jointHardness.hardness[4]);
  PLOT("module:KickEngine:JointDataReqHard5", JDReq.jointHardness.hardness[5]);
  PLOT("module:KickEngine:JointDataReqHard6", JDReq.jointHardness.hardness[6]);
  PLOT("module:KickEngine:JointDataReqHard7", JDReq.jointHardness.hardness[7]);
  PLOT("module:KickEngine:JointDataReqHard8", JDReq.jointHardness.hardness[8]);
  PLOT("module:KickEngine:JointDataReqHard9", JDReq.jointHardness.hardness[9]);
  PLOT("module:KickEngine:JointDataReqHard10", JDReq.jointHardness.hardness[10]);
  PLOT("module:KickEngine:JointDataReqHard11", JDReq.jointHardness.hardness[11]);
  PLOT("module:KickEngine:JointDataReqHard12", JDReq.jointHardness.hardness[12]);
  PLOT("module:KickEngine:JointDataReqHard13", JDReq.jointHardness.hardness[13]);
  PLOT("module:KickEngine:JointDataReqHard14", JDReq.jointHardness.hardness[14]);
  PLOT("module:KickEngine:JointDataReqHard15", JDReq.jointHardness.hardness[15]);
  PLOT("module:KickEngine:JointDataReqHard16", JDReq.jointHardness.hardness[16]);
  PLOT("module:KickEngine:JointDataReqHard17", JDReq.jointHardness.hardness[17]);
  PLOT("module:KickEngine:JointDataReqHard18", JDReq.jointHardness.hardness[18]);
  PLOT("module:KickEngine:JointDataReqHard19", JDReq.jointHardness.hardness[19]);
  PLOT("module:KickEngine:JointDataReqHard20", JDReq.jointHardness.hardness[20]);
  PLOT("module:KickEngine:JointDataReqHard21", JDReq.jointHardness.hardness[21]);
  PLOT("module:KickEngine:JointDataReqHard22", JDReq.jointHardness.hardness[22]);
  PLOT("module:KickEngine:JointDataReqHard23", JDReq.jointHardness.hardness[23]);
  PLOT("module:KickEngine:JointDataReqHard24", JDReq.jointHardness.hardness[24]);
}

void KickEngineData::ModifyData(const KickRequest& br,
                                JointRequest& kickEngineOutput,
                                std::vector<KickEngineParameters>& params) {
  auto& p = params.back();
  MODIFY("module:KickEngine:newKickMotion", p);
  strcpy(p.name, "newKick");
  MODIFY("module:KickEngine:px", gyroP.x);
  MODIFY("module:KickEngine:dx", gyroD.x);
  MODIFY("module:KickEngine:ix", gyroI.x);
  MODIFY("module:KickEngine:py", gyroP.y);
  MODIFY("module:KickEngine:dy", gyroD.y);
  MODIFY("module:KickEngine:iy", gyroI.y);

  MODIFY("module:KickEngine:formMode", formMode);
  MODIFY("module:KickEngine:lFootTraOff", limbOff[Phase::leftFootTra]);
  MODIFY("module:KickEngine:rFootTraOff", limbOff[Phase::rightFootTra]);
  MODIFY("module:KickEngine:lFootRotOff", limbOff[Phase::leftFootRot]);
  MODIFY("module:KickEngine:rFootRotOff", limbOff[Phase::rightFootRot]);
  MODIFY("module:KickEngine:lHandTraOff", limbOff[Phase::leftArmTra]);
  MODIFY("module:KickEngine:rHandTraOff", limbOff[Phase::rightArmTra]);
  MODIFY("module:KickEngine:lHandRotOff", limbOff[Phase::leftHandRot]);
  MODIFY("module:KickEngine:rHandRotOff", limbOff[Phase::rightHandRot]);

  // Plot com stabilizing
  PLOT("module:KickEngine:comy", robotModel.centerOfMass.y);
  PLOT("module:KickEngine:diffy", actualDiff.y);
  PLOT("module:KickEngine:refy", ref.y);

  PLOT("module:KickEngine:comx", robotModel.centerOfMass.x);
  PLOT("module:KickEngine:diffx", actualDiff.x);
  PLOT("module:KickEngine:refx", ref.x);

  PLOT("module:KickEngine:lastdiffy", toDegrees(lastBody.y));
  PLOT("module:KickEngine:bodyErrory", toDegrees(bodyError.y));

  const Pose3D& torso = toLeftSupport ? comRobotModel.limbs[Limbs::footLeft] : comRobotModel.limbs[Limbs::footRight];
  const Vector3<> com = torso.rotation.invert() * comRobotModel.centerOfMass;
  PLOT("module:KickEngine:comrotx", com.x);
  PLOT("module:KickEngine:comroty", com.y);

  PLOT("module:KickEngine:balanceSumx", balanceSum.x);
  PLOT("module:KickEngine:balanceSumy", balanceSum.y);

  PLOT("module:KickEngine:bodyanglex", bodyAngle.x / pi * 180);
  PLOT("module:KickEngine:bodyangley", bodyAngle.y / pi * 180);

  PLOT("module:KickEngine:rightFootTrap1", currentParameters.phaseParameters[3].controlPoints[Phase::rightFootTra][1].z);
  PLOT("module:KickEngine:rightFootTrap2", currentParameters.phaseParameters[3].controlPoints[Phase::rightFootTra][2].z);

  PLOT("module:KickEngine:originall", positions[Phase::leftFootTra].y);
  PLOT("module:KickEngine:originalr", positions[Phase::rightFootTra].y);
  PLOT("module:KickEngine:preview", yStateLeft[0]);
  PLOT("module:KickEngine:groundtruth", previewRefPointLookAhead[0]);

  DECLARE_PLOT("module:KickEngine:actualDiffY");
  DECLARE_PLOT("module:KickEngine:comY");
  DECLARE_PLOT("module:KickEngine:refY");
  DECLARE_PLOT("module:KickEngine:balanceY");
  DECLARE_PLOT("module:KickEngine:leftreq");
  DECLARE_PLOT("module:KickEngine:rightreq");

  //

  for (int i = 0; i < Phase::numOfLimbs; i++) {
    const int stiffness = limbOff[i] ? 0 : 85;
    switch (static_cast<Phase::Limb>(i)) {
    case Phase::leftFootTra:
      kickEngineOutput.jointHardness.hardness[JointData::LHipRoll] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::LHipPitch] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::LKneePitch] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::LAnklePitch] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::LAnkleRoll] = stiffness;
      break;
    case Phase::rightFootTra:
      kickEngineOutput.jointHardness.hardness[JointData::RHipRoll] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::RHipPitch] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::RKneePitch] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::RAnklePitch] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::RAnkleRoll] = stiffness;
      break;
    case Phase::leftFootRot:
      kickEngineOutput.jointHardness.hardness[JointData::LAnklePitch] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::LAnkleRoll] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::RHipYawPitch] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::LHipYawPitch] = stiffness;
      break;
    case Phase::rightFootRot:
      kickEngineOutput.jointHardness.hardness[JointData::RAnklePitch] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::RAnkleRoll] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::RHipYawPitch] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::LHipYawPitch] = stiffness;
      break;
    case Phase::leftArmTra:
      kickEngineOutput.jointHardness.hardness[JointData::LShoulderPitch] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::LShoulderRoll] = stiffness;
      break;
    case Phase::rightArmTra:
      kickEngineOutput.jointHardness.hardness[JointData::RShoulderPitch] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::RShoulderRoll] = stiffness;
      break;
    case Phase::leftHandRot:
      kickEngineOutput.jointHardness.hardness[JointData::LElbowRoll] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::LElbowYaw] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::LWristYaw] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::LHand] = stiffness;
      break;
    case Phase::rightHandRot:
      kickEngineOutput.jointHardness.hardness[JointData::RElbowRoll] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::RElbowYaw] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::RWristYaw] = stiffness;
      kickEngineOutput.jointHardness.hardness[JointData::RHand] = stiffness;
      break;
    }
  }
  phaseNumberPrevious = phaseNumber;
}

void KickEngineData::calcPositions(const TorsoMatrix& torsoMatrix, KickRequest::KickMotionID& kickMotionType) {
  if (kickMotionType != KickRequest::fastKick) {
    extendControlPoints(phaseNumber);
  }
  updateRemainingDuration();
  updatePreviewRefPoints();
  for (int i = 0; i < Phase::numOfLimbs; ++i) {
    if (i == Phase::rightFootTra && phaseNumber == currentParameters.kickSwingPhaseID) {
      positions[i] = currentParameters.getPositionWithStraightLineExtension(phase, phaseNumber, i, omniKickParams);
    } else {
      positions[i] = currentParameters.getPosition(phase, phaseNumber, i);
    }
  }
  if (omniKickParams.usePreviewController) {
    stateEvolution(yStateLeft);
    stateEvolution(yStateRight);
    positions[Phase::leftFootTra].y = yStateLeft[0];
    if (phaseNumber <= 1 || phaseNumber >= 4) {
      if (phaseNumberPrevious == 3) {
        yStateRight = {positions[Phase::rightFootTra].y + 100, 0.0f, 0.0f};
      }
      positions[Phase::rightFootTra].y = yStateRight[0] - 100;
    }
  }

  bool adjustKick = adjustFastKickHack(torsoMatrix);
  if (adjustKick) {
    if (!fastKickEndAdjusted) {
      adjustedZValue = positions[Phase::rightFootTra].z +
                       5.f; // the edges of the foot should be used to get the right foot height. this is a working hack tho.
      adjustedXValue = positions[Phase::rightFootTra].x;
      fastKickEndAdjusted = true;
    } else {
      positions[Phase::rightFootTra].z = adjustedZValue;
      positions[Phase::rightFootTra].x = adjustedXValue;
    }
  }
  if (!currentParameters.ignoreHead) {
    head = currentParameters.getHeadRefPosition(phase, phaseNumber);
  }

  Vector2<> params = currentParameters.getComRefPosition(phase, phaseNumber);
  ref = Vector3<>(params.x, params.y, (toLeftSupport) ? positions[Phase::leftFootTra].z : positions[Phase::rightFootTra].z);
}

void KickEngineData::setExecutedKickRequest(KickRequest& br) {
  br.mirror = currentKickRequest.mirror;
  br.armsBackFix = currentKickRequest.armsBackFix;
  br.kickMotionType = currentKickRequest.kickMotionType;
}

void KickEngineData::initData(const FrameInfo& frame,
                              KickRequest& kr,
                              std::vector<KickEngineParameters>& params,
                              const JointData& ja,
                              const TorsoMatrix& torsoMatrix,
                              JointRequest& jointRequest,
                              const RobotDimensions& rd,
                              const MassCalibration& mc,
                              const DamageConfiguration& theDamageConfiguration,
                              const BallModel& theBallModel,
                              const omniKickParameters& theOmniKickParameters) {
  VERIFY(getMotionIDByName(kr, params));
  omniKickParams = theOmniKickParameters;
  cycletime = 0.012f; // frame.cycleTime;
  for (int i = 0; i < Phase::numOfLimbs; i++) {
    controllPointExtendedFlags[i] = false;
  }
  previewRefPointLookAhead.resize(omniKickParams.previewHorizon);
  for (int i = 0; i < omniKickParams.previewHorizon - 1; i++) {
    previewRefPointLookAhead[i] = 50; // default is 50
  }
  timeStep = omniKickParams.timeStepDuration;
  A1 = {1.0f, timeStep, 0.5f * timeStep * timeStep};
  A2 = {0.0f, 1.0f, timeStep};
  A3 = {0.0f, 0.0f, 1.0f};
  B = {0.1666f * timeStep * timeStep * timeStep, 0.5f * timeStep * timeStep, timeStep};

  phase = 0.f;
  phaseNumber = 0;
  timestamp = frame.time;
  timestampKickStarted = frame.time;
  currentParameters = params[motionID];
  toLeftSupport = currentParameters.standLeft;
  adjustedZValue = 0.f;
  adjustedXValue = 0.f;
  fastKickEndAdjusted = 0.f;
  lastZDif = 0.f;
  lastXDif = 0.f;

  kickAngle = kr.kickAngle;
  kickPower = kr.kickPower;
  bool mirror = decideKickLeg(kr.kickAngle, theBallModel);
  kr.mirror = mirror;
  currentParameters.mirror = kr.mirror;
  ref = Vector3<>();
  actualDiff = ref.toVec2();
  calculateOrigins(kr, ja, torsoMatrix, rd);
  currentParameters.initFirstPhase(origins,
                                   Vector2<>(ja.angles[JointData::HeadPitch],
                                             (kr.mirror) ? -ja.angles[JointData::HeadYaw] : ja.angles[JointData::HeadYaw]));
  calcPositions(torsoMatrix, kr.kickMotionType);

  float angleY = toLeftSupport ? -robotModel.limbs[Limbs::footLeft].rotation.invert().getYAngle()
                               : -robotModel.limbs[Limbs::footRight].rotation.invert().getYAngle();
  float angleX = toLeftSupport ? -robotModel.limbs[Limbs::footLeft].rotation.invert().getXAngle()
                               : -robotModel.limbs[Limbs::footRight].rotation.invert().getXAngle();
  if (kr.mirror) {
    angleX *= -1.f;
  }

  bodyAngle = Vector2<>(angleX, angleY);
  calcJoints(jointRequest, rd, theDamageConfiguration);
  comRobotModel.setJointData(jointRequest.jointAngles, rd, mc);
  const Pose3D& torso = toLeftSupport ? comRobotModel.limbs[Limbs::footLeft] : comRobotModel.limbs[Limbs::footRight];
  const Vector3<> com = torso.rotation.invert() * comRobotModel.centerOfMass;

  // this calculates invert of pid com control -> getting com to max pos at start -> beeing rotated as before engine
  float foot = toLeftSupport ? origins[Phase::leftFootTra].z : origins[Phase::rightFootTra].z;
  float height = comRobotModel.centerOfMass.z - foot;

  if (omniKickParams.newBalanceMethod ||
      (kr.kickMotionType == KickRequest::omniKickDefault || kr.kickMotionType == KickRequest::omniKickLong)) {
    balanceSum = Vector2<>(0, 0);
  } else {
    balanceSum.x = std::tan(angleY - Constants::pi) * height;
    balanceSum.x /= currentParameters.kiy;
    balanceSum.y = std::tan(angleX - Constants::pi) * height;
    balanceSum.y /= -currentParameters.kix;
  }

  currentParameters.initFirstPhaseLoop(
    origins,
    Vector2<>(com.x, com.y),
    Vector2<>(ja.angles[JointData::HeadPitch],
              (kr.mirror) ? -ja.angles[JointData::HeadYaw] : ja.angles[JointData::HeadYaw]));

  if (!wasActive) {
    // origin = Vector2<>();
    gyro = Vector2<>();
    lastGyroLeft = Vector2<>();
    lastGyroRight = Vector2<>();
    gyroErrorLeft = Vector2<>();
    gyroErrorRight = Vector2<>();
    bodyError = Vector2<>();
    lastBody = Vector2<>();
    lastCom = Vector3<>();

    for (int i = 0; i < JointData::numOfJoints; i++) {
      lastBalancedJointRequest.jointAngles.angles[i] = ja.angles[i];
    }
  }
  for (unsigned int i = 0; i < kr.dynPoints.size(); i++) {
    if (kr.dynPoints[i].phaseNumber == phaseNumber) {
      addDynPoint(kr.dynPoints[i], torsoMatrix);
    }
  }

  lElbowFront = origins[Phase::leftHandRot].x > pi_4;
  rElbowFront = origins[Phase::rightHandRot].x < -pi_4;

  if (kr.armsBackFix) // quick hack to not break arms while they are on the back
  {
    if (lElbowFront) {
      addDynPoint(DynPoint(Phase::leftHandRot, 0, Vector3<>(pi_2, -pi_4, 0)), torsoMatrix);
    }

    if (rElbowFront) {
      addDynPoint(DynPoint(Phase::rightHandRot, 0, Vector3<>(-pi_2, pi_4, 0)), torsoMatrix);
    }
  }
}

bool KickEngineData::decideKickLeg(float kickAngle, const BallModel& theBallModel) {
  // decide to use mirror or not based on the provided kickAngle and relballposition
  Vector3<> leftFootPos = currentParameters.phaseParameters[0].controlPoints[Phase::leftFootTra][1];
  Vector3<> rightFootPos = currentParameters.phaseParameters[0].controlPoints[Phase::rightFootTra][1];
  Vector3<> relBallPos3 = {theBallModel.estimate.position.x, theBallModel.estimate.position.y, leftFootPos.z + 10};
  bool kickWithRight = true;
  if (relBallPos3.y > leftFootPos.y) {
    kickWithRight = false;
  } else {
    if (relBallPos3.y < rightFootPos.y) {
      kickWithRight = true;
    } else {
      float angleLeftFoot2Ball = std::atan2(relBallPos3.y - leftFootPos.y, relBallPos3.x - leftFootPos.x);
      float angleRightFoot2Ball = std::atan2(relBallPos3.y - rightFootPos.y, relBallPos3.x - rightFootPos.x);
      if (std::abs(angleRightFoot2Ball - kickAngle) <= std::abs(angleLeftFoot2Ball - kickAngle)) {
        kickWithRight = true;
      } else {
        kickWithRight = false;
      }
    }
  }
  return !kickWithRight;
}

void KickEngineData::updateBallPosition(const BallModel& theBallModel) {
  relBallPosition = theBallModel.estimate.position;
  // relBallPosition = theBallModel.lastPerception;
  if (theBallModel.timeSinceLastSeen < 1500) {
    validBallPosition = true;
  } else {
    validBallPosition = false;
  }
}

void KickEngineData::setEngineActivation(const float& ratio) {
  willBeLeft = (ratio < 1.f && lastRatio > ratio);
  wasActive = (ratio != 0.f && motionID > -1);
  startComp = (ratio != 0.f && lastRatio <= ratio);
  lastRatio = ratio;
}

bool KickEngineData::activateNewMotion(const KickRequest& br, const bool& isLeavingPossible) {
  if (!wasActive || (br.kickMotionType != currentKickRequest.kickMotionType && isLeavingPossible)) {
    return true;
  } else if (br.kickMotionType == currentKickRequest.kickMotionType && br.mirror == currentKickRequest.mirror) {
    currentKickRequest = br; // update KickRequest when it is compatible to the current motion
  }

  return false;
}

bool KickEngineData::sitOutTransitionDisturbance(bool& compensate,
                                                 bool& compensated,
                                                 const InertiaSensorData& id,
                                                 KickEngineOutput& kickEngineOutput,
                                                 const JointRequest& theJointRequest,
                                                 const FrameInfo& frame) {
  if (compensate) {
    if (!startComp) {
      timestamp = frame.time;
      gyro = Vector2<>();
      lastGyroLeft = Vector2<>();
      lastGyroRight = Vector2<>();
      gyroErrorLeft = Vector2<>();
      gyroErrorRight = Vector2<>();
      bodyError = Vector2<>();
      lastBody = Vector2<>();
      lastCom = Vector3<>();
      motionID = -1;

      kickEngineOutput.isLeavingPossible = false;
      for (int i = 0; i < JointData::numOfJoints; ++i) {
        lastBalancedJointRequest.jointAngles.angles[i] = theJointRequest.jointAngles.angles[i];
        compenJoints.jointAngles.angles[i] = theJointRequest.jointAngles.angles[i];
        kickEngineOutput.jointRequest.jointHardness.hardness[i] = theJointRequest.jointHardness.hardness[i];
        kickEngineOutput.jointRequest.jointAngles.angles[i] = compenJoints.jointAngles.angles[i];
      }
    }

    int time = frame.getTimeSince(timestamp);
    if ((std::abs(id.acc.x) < 0.6f && std::abs(id.acc.y) < 0.6f && std::abs(id.gyro.x) < 0.1f &&
         std::abs(id.gyro.y) < 0.1f) ||
        time > 1000) {
      int timeStable = frame.getTimeSince(timestampStable);
      if (timeStable > 100 || time > 1000) {
        compensate = false;
        compensated = true;
        return true;
      } else {
        return false;
      }

    } else {
      return false;
    }
  }

  return true;
}

void KickEngineData::updateRemainingDuration() {
  remainingDurationToPhase.resize(currentParameters.numberOfPhases);
  remainingDurationToPhase[0] = -static_cast<int>(timeSinceKickStarted) + currentParameters.phaseParameters[0].duration;
  for (int i = 1; i < currentParameters.numberOfPhases; i++) {
    remainingDurationToPhase[i] = remainingDurationToPhase[i - 1] + currentParameters.phaseParameters[i].duration;
  }
}

int KickEngineData::checkPhaseNumberLimit(int phaseNumber) {
  return (phaseNumber < currentParameters.numberOfPhases) ? phaseNumber : currentParameters.numberOfPhases - 1;
}

void KickEngineData::updatePreviewRefPoints() {
  float refPointNext =
    currentParameters.phaseParameters[checkPhaseNumberLimit(phaseNumber + 1)].controlPoints[Phase::leftFootTra][2].y;
  float refPointNext2 =
    currentParameters.phaseParameters[checkPhaseNumberLimit(phaseNumber + 2)].controlPoints[Phase::leftFootTra][2].y;
  int timeStep = omniKickParams.timeStepDuration * 1000;
  int stepsToNextPhase = (int)(remainingDurationToPhase[checkPhaseNumberLimit(phaseNumber)] / timeStep);
  int stepsToNext2Phase = (int)(remainingDurationToPhase[checkPhaseNumberLimit(phaseNumber + 1)] / timeStep);
  if (stepsToNextPhase < omniKickParams.previewHorizon && stepsToNextPhase >= 0) {
    for (int i = omniKickParams.previewHorizon - 1; i >= stepsToNextPhase; i--) {
      previewRefPointLookAhead[i] = refPointNext;
    }
  }
  if (stepsToNext2Phase < omniKickParams.previewHorizon && stepsToNextPhase >= 0) {
    previewRefPointLookAhead[stepsToNext2Phase] = refPointNext2;
  }
}

void KickEngineData::stateEvolution(std::vector<float>& state) {
  std::vector<float> state_new = {0.0f, 0.0f, 0.0f};
  float uFeedBack = std::inner_product(omniKickParams.lqrGain.begin(), omniKickParams.lqrGain.end(), state.begin(), 0.0f);
  float uPreview = std::inner_product(omniKickParams.previewControllerGain.begin(),
                                      omniKickParams.previewControllerGain.end(),
                                      previewRefPointLookAhead.begin(),
                                      0.0f);
  float u = -uFeedBack - uPreview;
  state_new[0] = std::inner_product(A1.begin(), A1.end(), state.begin(), 0.0) + B[0] * u;
  state_new[1] = std::inner_product(A2.begin(), A2.end(), state.begin(), 0.0) + B[1] * u;
  state_new[2] = std::inner_product(A3.begin(), A3.end(), state.begin(), 0.0) + B[2] * u;
  state = state_new;
}

void KickEngineData::extendControlPoints(const int& phaseNumber) {
  for (int limb = 0; limb < Phase::numOfLimbs; ++limb) {
    if (validBallPosition && (currentKickRequest.kickMotionType == KickRequest::omniKickDefault ||
                              currentKickRequest.kickMotionType == KickRequest::omniKickLong)) {
      if (currentParameters.kickSwingPhaseID == 3) { // omniKickDefault has swing phase 2 and omniKickLong has swing phase 3
        extendControlPointsSwingBackPhase(phaseNumber, limb);
      }
      extendControlPointsKickPhase(phaseNumber, limb);
    }
  }
}

void KickEngineData::extendControlPointsSwingBackPhase(const int& phaseNumber, const int& limb) {
  if (phaseNumber == currentParameters.kickSwingPhaseID - 1 && limb == Phase::rightFootTra &&
      !controllPointExtendedFlags[phaseNumber]) {
    std::vector<Phase>& phaseParameters = currentParameters.phaseParameters;
    controllPointExtendedFlags[phaseNumber] = true;
    Vector3<> p1;
    Vector3<> currentSupportFoot = phaseParameters[phaseNumber].controlPoints[Phase::leftFootTra][1];
    Vector3<> originOffset = currentParameters.footOrigin - currentSupportFoot;
    Vector3<> kickFootOrigin = phaseParameters[1].controlPoints[limb][1];
    Vector3<> relBallPosition3;
    Vector2<> kickMiddleRefPoint = {kickFootOrigin.x, kickFootOrigin.y};

    float swingBackHeight = phaseParameters[phaseNumber].controlPoints[limb][1].z;
    float swingBackDistance = 120;
    int mirrorFactor = 1;
    if (currentParameters.mirror) {
      mirrorFactor = -1;
    }
    originOffset.y = mirrorFactor * originOffset.y;

    relBallPosition3 = {relBallPosition.x, mirrorFactor * (relBallPosition.y - originOffset.y), kickFootOrigin.z};
    float angleToBall = std::atan2(relBallPosition3.y - kickMiddleRefPoint.y, relBallPosition3.x - kickMiddleRefPoint.x);
    float swingBackAngle = 0.3 * angleToBall + 0.7 * mirrorFactor * kickAngle;
    Vector2<> swingBackDirection = {-std::cos(swingBackAngle), -std::sin(swingBackAngle)};
    if (swingBackAngle < 0) {
      swingBackDistance = std::min(swingBackDistance, (25) / std::abs(std::sin(swingBackAngle)));
    }
    Vector2<> swingBackPosition1 = {kickMiddleRefPoint + swingBackDirection * swingBackDistance / 2};
    Vector2<> swingBackPosition2 = {kickMiddleRefPoint + swingBackDirection * swingBackDistance};

    phaseParameters[phaseNumber].controlPoints[limb][1] = {swingBackPosition1.x, swingBackPosition1.y, swingBackHeight};
    phaseParameters[phaseNumber].controlPoints[limb][2] = {swingBackPosition2.x, swingBackPosition2.y, swingBackHeight};
  }
}

void KickEngineData::extendControlPointsKickPhase(const int& phaseNumber, const int& limb) {
  if (phaseNumber == currentParameters.kickSwingPhaseID && limb == Phase::rightFootTra &&
      !controllPointExtendedFlags[phaseNumber]) {
    std::vector<Phase>& phaseParameters = currentParameters.phaseParameters;
    controllPointExtendedFlags[phaseNumber] = true;
    float supportFootFrontOffset = omniKickParams.supportfootCollisionOffset.x;
    float supportFootSideCollisionOffset = omniKickParams.supportfootCollisionOffset.y;

    float supportFootSideMaxReach = omniKickParams.maxFootReach.y;
    float kickReferencePointOffset = omniKickParams.kickReferencePointOffset.x; // values need fine turning
    const float kickFootEffectiveSize = 10;
    const float ballRadius = 15;

    Vector3<> p1, p2;
    Vector3<> kickFootOrigin = phaseParameters[1].controlPoints[limb][1];
    Vector3<> currentSupportFoot = phaseParameters[phaseNumber].controlPoints[Phase::leftFootTra][1];
    Vector2<> kickMiddleRefPoint =
      kickFootOrigin.toVec2() + omniKickParams.kickDirectionRefPoint - omniKickParams.kickReferencePointOffset;
    Vector3<> relBallPosition3;
    Vector3<> kickDirection;
    const float middlePointRefDistance = 180;
    Vector3<> originOffset = currentParameters.footOrigin - currentSupportFoot;
    int mirrorFactor = 1;
    if (currentParameters.mirror) {
      mirrorFactor = -1;
    }

    originOffset.y = mirrorFactor * originOffset.y;

    relBallPosition3 = {
      relBallPosition.x - kickReferencePointOffset, mirrorFactor * (relBallPosition.y - originOffset.y), kickFootOrigin.z};

    if ((relBallPosition3 - kickFootOrigin).abs() > omniKickParams.maxFootReach.x) {
      relBallPosition3 = kickFootOrigin + (relBallPosition3 - kickFootOrigin) / (relBallPosition3 - kickFootOrigin).abs() *
                                            omniKickParams.maxFootReach.x;
    }
    relBallPosition3.z += omniKickParams.kickHeight;
    float minAllowedKickAngle =
      std::atan2(relBallPosition3.y - kickMiddleRefPoint.y, relBallPosition3.x - kickMiddleRefPoint.x);
    float maxAllowedKickAngle = pi / 2;
    if ((relBallPosition3 - kickFootOrigin).y >= 20) {
      kickAngle = std::min(maxAllowedKickAngle, std::max(mirrorFactor * kickAngle, minAllowedKickAngle));

      // Avoid collision with the supportfoot
      if (relBallPosition3.y >= currentSupportFoot.y + supportFootSideMaxReach) {
        relBallPosition3.y = currentSupportFoot.y + supportFootSideMaxReach;
      }
      if (relBallPosition3.x < currentSupportFoot.x + supportFootFrontOffset) {
        relBallPosition3.x = currentSupportFoot.x + supportFootFrontOffset;
      }
    } else {
      kickAngle = mirrorFactor * kickAngle;
    }
    if (relBallPosition3.x < omniKickParams.minimumDistance) {
      relBallPosition3.x = omniKickParams.minimumDistance;
    }
    kickDirection = {std::cos(kickAngle), std::sin(kickAngle), 0};
    relBallPosition3 = relBallPosition3 - kickDirection * (ballRadius + kickFootEffectiveSize);
    kickDirection.z = .0;
    Vector3<> middlePoint = relBallPosition3 - kickDirection * middlePointRefDistance;
    // This check the collision condition for the middle point with respect the the support foot
    if ((middlePoint.y - currentSupportFoot.y) >= supportFootSideCollisionOffset) {
      middlePoint = relBallPosition3 -
                    kickDirection * std::abs(relBallPosition3.y - (currentSupportFoot.y + supportFootSideCollisionOffset)) /
                      std::abs(kickDirection.y);
    }
    // This check the collision condition for the end point with respect the the support foot
    if ((relBallPosition3.y - currentSupportFoot.y) >= supportFootSideCollisionOffset) {
      relBallPosition3 =
        relBallPosition3 - kickDirection *
                             std::abs(relBallPosition3.y - (currentSupportFoot.y + supportFootSideCollisionOffset)) /
                             std::abs(kickDirection.y);
    }
    p1 = middlePoint;
    p2 = relBallPosition3;
    phaseParameters[phaseNumber].controlPoints[limb][1] = p1;
    phaseParameters[phaseNumber].controlPoints[limb][2] = p2;
    if (omniKickParams.useDuration) {
      phaseParameters[phaseNumber].duration =
        (1 - kickPower) * omniKickParams.durationModeRange.max + kickPower * omniKickParams.durationModeRange.min;
    } else {
      float kickTraLength = (p1 - phaseParameters[phaseNumber - 1].controlPoints[limb][2]).abs() + (p2 - p1).abs();
      float kickVelocity = (1 - kickPower) * omniKickParams.speedModeRange.min +
                           kickPower * omniKickParams.speedModeRange.max; // kick speed in mm/s
      phaseParameters[phaseNumber].duration = kickTraLength / kickVelocity * 1000;
    }
  }
}

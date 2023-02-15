/**
 * @file NaoKinematic.cpp
 *
 * This file implements the inverse kinematic.
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original authors are <A href="mailto:Stefan.Czarnetzki@uni-dortmund.de">Stefan Czarnetzki</A>,
 * <ahref="mailto:oliver.urbann@tu-dortmund.de"> Oliver Urbann</a>
 */

//#define LOGGING
#include "Tools/Motion/InverseKinematic.h"
#include "NaoKinematic.h"
#include "Tools/DortmundWalkingEngine/StepData.h"
#ifndef WALKING_SIMULATOR
//#include "Core/System/GTAssert.h"
#include "Core/Math/BHMath.h"
#include "Core/Debugging/Debugging.h"
#include "Core/Streams/InStreams.h"
#include "Core/Settings.h"
#else
#include "math/Pose3D.h"
#include "math/Common.h"
#include "csvlogger.h"
#endif

NaoKinematic::NaoKinematic() {}

NaoKinematic::~NaoKinematic() {}

Vector3<> NaoKinematic::checkConstraints(Vector3<> lf, float lfr, Vector3<> rf, float rfr, bool correctleft) {
  Vector2<> checkPoints[2], constraintPoint, innerPoint;
  Vector3<> cf3D;

  checkPoints[0].x = theRobotDimensions.footFront;
  checkPoints[1].x = theRobotDimensions.footBack;
  constraintPoint.x = theRobotDimensions.footFront;
  innerPoint.x = theRobotDimensions.footFront;
  float footInner = theRobotDimensions.footInner - 10;
  float footOuter = theRobotDimensions.footOuter - 10;
  if (correctleft) {
    checkPoints[0].y = -footInner;
    checkPoints[0].rotate(lfr);
    checkPoints[0].x += lf.x;
    checkPoints[0].y += lf.y;

    checkPoints[1].y = -footInner;
    checkPoints[1].rotate(lfr);
    checkPoints[1].x += lf.x;
    checkPoints[1].y += lf.y;

    constraintPoint.y = footInner;
    constraintPoint.rotate(rfr);
    constraintPoint.x += rf.x;
    constraintPoint.y += rf.y;

    innerPoint.y = -footOuter;
    innerPoint.rotate(rfr);
    innerPoint.x += rf.x;
    innerPoint.y += rf.y;

    cf3D = lf;
  } else {
    // right foot will be checked and moved

    checkPoints[0].y = footInner;
    checkPoints[0].rotate(rfr);
    checkPoints[0].x += rf.x;
    checkPoints[0].y += rf.y;

    checkPoints[1].y = footInner;
    checkPoints[1].rotate(rfr);
    checkPoints[1].x += rf.x;
    checkPoints[1].y += rf.y;

    constraintPoint.y = -footInner;
    constraintPoint.rotate(lfr);
    constraintPoint.x += lf.x;
    constraintPoint.y += lf.y;

    innerPoint.y = footOuter;
    innerPoint.rotate(lfr);
    innerPoint.x += lf.x;
    innerPoint.y += lf.y;

    cf3D = rf;
  }

  Vector2<> a = innerPoint - constraintPoint;
  Vector2<> b[2];
  b[0] = checkPoints[0] - constraintPoint;
  b[1] = checkPoints[1] - constraintPoint;

  a.normalize();

  float dot[2];
  dot[0] = a.x * b[0].x + a.y * b[0].y;
  dot[1] = a.x * b[1].x + a.y * b[1].y;

  int largestIndex;
  if (dot[0] > dot[1]) {
    largestIndex = 0;
  } else {
    largestIndex = 1;
  }

  Vector2<> cf(cf3D.x, cf3D.y);

  if (dot[largestIndex] > 0) {
    cf -= a * dot[largestIndex];
  }

  cf3D.x = cf.x;
  cf3D.y = cf.y;
  return cf3D;
}

bool NaoKinematic::calcLegJoints(JointData::Joint whichSideJoint0,
                                 const Vector3<>& position,
                                 const Vector3<>& rotation,
                                 float t0,
                                 JointRequest& jointRequest,
                                 const RobotDimensions& robotDimensions) {
  bool result = true;

  float footHeight = robotDimensions.heightLeg5Joint;
  float tibiaLength = robotDimensions.lowerLegLength;
  float thighLength = robotDimensions.upperLegLength;
  float hipOffsetY = robotDimensions.yHipOffset;
  float hipOffsetZ = 0; // 85;

#ifdef WALKING_SIMULATOR
// hipOffsetZ = 85;
#endif

  float right = static_cast<float>((whichSideJoint0 == JointData::LHipYawPitch) ? -1 : 1);

  float tiltAngleOfFirstJointAxis = -pi / 2 + right * pi / 4;

  // Rotate around X to create HipYawPitch axis
  Matrix3x3<> M1 = Matrix3x3<>();
  M1[1][1] = std::cos(tiltAngleOfFirstJointAxis);
  M1[1][2] = std::sin(tiltAngleOfFirstJointAxis);
  M1[2][1] = -std::sin(tiltAngleOfFirstJointAxis);
  M1[2][2] = std::cos(tiltAngleOfFirstJointAxis);

  // When created rotate back
  Matrix3x3<> M2 = Matrix3x3<>();
  M2[1][1] = std::cos(tiltAngleOfFirstJointAxis);
  M2[1][2] = -std::sin(tiltAngleOfFirstJointAxis);
  M2[2][1] = std::sin(tiltAngleOfFirstJointAxis);
  M2[2][2] = std::cos(tiltAngleOfFirstJointAxis);

  float rx = rotation.x;

  // BEGIN	footRotX=rotMatrixAroundX(rx);
  //			footRot=rotMatrixAroundY(ry);
  //			footHeightVec=footRot*footRotX*[0;0;46];
  RotationMatrix footRot, footRotX;
  Vector3<> footHeightVec(0, 0, footHeight);
  footRotX.rotateX(rx);
  footRot.rotateY(rotation.y);
  footHeightVec = footRot * footRotX * footHeightVec;

  // END

  // BEGIN p = M1 * rotMatrixAroundZ(-t0)*M2*([x;y;z]+[0;0;46]+[0;right*50;85]);
  RotationMatrix rotZ;
  rotZ.rotateZ(-t0);

  Vector3<> p(position.x, position.y + right * hipOffsetY, position.z + hipOffsetZ);
  p = p + footHeightVec;
  p = M1 * rotZ * M2 * p;
  // END p = M1 * rotMatrixAroundZ(-t0)*M2*([x;y;z]+[0;0;46]+[0;right*50;85]);

  float ysign = 1;

  float r = p.abs();
#if 0
    ASSERT(r <= tibiaLength + thighLength);
#else
  if (r > tibiaLength + thighLength) {
    // OUTPUT(idText, text, "Bein zu kurz!\r\n");
    p.normalize(tibiaLength + thighLength);
    r = tibiaLength + thighLength;
    // Vector3<>(0, 0, -hipOffsetZ) + M1 * rotZ * M2 * Vector3<>(
    result = false;
  }
#endif
  // Kosinussatz fuer das Knie
  float t3 = std::acos((sqr(tibiaLength) + sqr(thighLength) - sqr(r)) / (2 * tibiaLength * thighLength));
  float t1 = std::atan(p[1] / p[2]);

  Vector3<> p1(p);
  p1[0] = 0;
  float t2a = std::acos((sqr(thighLength) + sqr(r) - sqr(tibiaLength)) / (2 * thighLength * r));

  float soll;
  if (p[0] != 0) {
    soll = std::atan(p[0] / p1.abs());
  } else {
    soll = 0;
  }

  float t2 = t2a + soll;

  tiltAngleOfFirstJointAxis = -pi / 2 + pi / 4;

  M1[1][1] = std::cos(tiltAngleOfFirstJointAxis);
  M1[1][2] = std::sin(tiltAngleOfFirstJointAxis);
  M1[2][1] = -std::sin(tiltAngleOfFirstJointAxis);
  M1[2][2] = std::cos(tiltAngleOfFirstJointAxis);

  M2[1][1] = std::cos(tiltAngleOfFirstJointAxis);
  M2[1][2] = -std::sin(tiltAngleOfFirstJointAxis);
  M2[2][1] = std::sin(tiltAngleOfFirstJointAxis);
  M2[2][2] = std::cos(tiltAngleOfFirstJointAxis);

  // BEGIN R=M1*rotMatrixAroundZ(t0)*M2; R=R*rotMatrixAroundX(ysign*t1)*rotMatrixAroundY(-t2)*rotMatrixAroundY(pi-t3);
  Matrix3x3<> R = Matrix3x3<>();
  RotationMatrix rotZ2, rotY4, rotX, rotY5;
  rotZ2.rotateZ(t0);
  rotY4.rotateY(-t2);
  rotX.rotateX(ysign * t1);
  rotY5.rotateY(pi - t3);
  R = M1 * rotZ2 * M2 * rotX * rotY4 * rotY5;
  // END R=M1*rotMatrixAroundZ(t0)*M2; R=R*rotMatrixAroundX(ysign*t1)*rotMatrixAroundY(-t2)*rotMatrixAroundY(pi-t3);

  // BEGIN schnitt=cross(R*[0;1;0], [0;0;1]);
  Vector3<> v3(0, 0, 1);
  Vector3<> v4(0, 1, 0);
  v4 = R * v4;
  Vector3<> schnitt(v4.cross(footRot * v3));
  // END schnitt=cross(R*[0;1;0], [0;0;1]);

  Vector3<> vek(R * v3);

  float t4 = -std::asin((schnitt * vek) / (schnitt.abs() * vek.abs()));

  // BEGIN R=R*rotMatrixAroundY(t4);
  RotationMatrix rotY6;
  rotY6.rotateY(t4);
  R = R * rotY6;
  // END R=R*rotMatrixAroundY(t4);

  // BEGIN schnitt=cross(R*[1;0;0], [0;0;1]);
  Vector3<> v5(1, 0, 0);
  schnitt = (R * v5).cross(footRot * v3);
  // END schnitt=cross(R*[1;0;0], [0;0;1]);

  vek = R * v3;

  // BEGIN t5=-asin((schnitt'*vek)/(norm(schnitt)*norm(vek)))+rx;
  float t5 = -std::asin((schnitt * vek) / (schnitt.abs() * vek.abs())) + right * rx;
  // END

  jointRequest.jointAngles.angles[whichSideJoint0 + 0] = (float)t0;      // [-90 ;  0 ] = [...
  jointRequest.jointAngles.angles[whichSideJoint0 + 1] = (float)(-t1);   // [-100; 25 ] = [...
  jointRequest.jointAngles.angles[whichSideJoint0 + 2] = (float)-t2;     // [-45 ; 25 ]
  jointRequest.jointAngles.angles[whichSideJoint0 + 3] = pi - (float)t3; // [ 0  ; 130]
  jointRequest.jointAngles.angles[whichSideJoint0 + 4] = (float)t4;      // [-75 ; 45 ]
  jointRequest.jointAngles.angles[whichSideJoint0 + 5] = (float)-t5;     // [-25 ; 45 ]
  return result;
}

bool NaoKinematic::calcLegJoints(JointData::Joint whichSideJoint0,
                                 const Vector3<>& position,
                                 const Vector3<>& rotation,
                                 JointRequest& jointRequest,
                                 const RobotDimensions& robotDimensions) {
  // to provide clearification about the constants in the robot dimensions:
  // (referring to the figure provided by aldebaran)
  // http://www.aldebaran-robotics.com/robocup/doc/docref/general/robocup/media/RobocupmDHDefintion.jpg
  // double footHeight =46;//= theRobotDimensions.heightLeg5Joint;
  // double tibiaLength =100;//= theRobotDimensions.lowerLegLength;
  // double thighLength =100;//= theRobotDimensions.upperLegLength;
  // double hipOffsetY =50;//= 0.5 * theRobotDimensions.lengthBetweenLegs;
  //// HipOffsetZ = ?
  // double hipOffsetZ = 0;//85;
  bool result = true;

  float footHeight = robotDimensions.heightLeg5Joint;
  float tibiaLength = robotDimensions.lowerLegLength;
  float thighLength = robotDimensions.upperLegLength;
  float hipOffsetY = robotDimensions.yHipOffset;
  float hipOffsetZ = 0; // 85;

#ifdef WALKING_SIMULATOR
// hipOffsetZ = 85;
#endif

  float sign = static_cast<float>((whichSideJoint0 == JointData::LHipYawPitch) ? -1 : 1);

  RotationMatrix rot;
  rot.rotateX(rotation.x);
  rot.rotateY(rotation.y);
  rot.rotateZ(rotation.z);

  Pose3D T(rot, position);

  //    T = T(Waist->J0) T0to5 T(J5->Foot)
  // => T0to5 = T(Waist->J0)^-1 T T(J5->Foot)^-1
  // with M1 = T(Waist->J0)^-1
  //      M2 = T(J5->Foot)^-1

  Pose3D M1(0, -sign * hipOffsetY, -hipOffsetZ);
  M1 = M1.invert();
  Pose3D M2(0, 0, -footHeight);
  M2 = M2.invert();

  Pose3D T0to5 = M1 * T * M2;

  // now let's calculate the angles
  float t0, t1, t2, t3, t4, t5;
  float angle1; // will be needed later

  float length =
    T0to5.translation.abs(); // sqrt( T0to5(1,4) *  T0to5(1,4) + T0to5(2,4) * T0to5(2,4) + T0to5(3,4) * T0to5(3,4) );

  if (length > thighLength + tibiaLength) {
    t3 = 0;
    angle1 = 0;
  } else {
    // first the knee angle by law of cosines
    float kneeAngle =
      std::acos((tibiaLength * tibiaLength + thighLength * thighLength - length * length) / (2 * thighLength * tibiaLength));
    t3 = pi - kneeAngle; // Will be a positive number. t3 might theoretically also be negative, but the knee joint can only
                         // be bent in one direction, so this is the correct one.

    angle1 = std::asin(thighLength * std::sin(kneeAngle) / length); // law of sines
  }

  Pose3D T0to5Inv = T0to5.invert();
  t5 = std::atan2(T0to5Inv.translation.y, T0to5Inv.translation.z); // atan2(T0to5Inv(2,4),T0to5Inv(3,4));
  t4 = std::atan2(
         -T0to5Inv.translation.x,
         std::sqrt(T0to5Inv.translation.y * T0to5Inv.translation.y + T0to5Inv.translation.z * T0to5Inv.translation.z)) -
       angle1;

  //    T0to5 = T0to2 * T2to3 * T3to4 * T4to5
  // => T0to2 = T0to5 * T4to5^-1 * T3to4^-1 * T2to3^-1

  // RotationMatrix dummy; // somehow the static call doesn't work; has to be investigated
  Pose3D T4to5Inv(RotationMatrix().rotateX(-t5));
  Pose3D T3to4Inv(RotationMatrix().rotateY(-t4));
  Pose3D T2to3Inv(RotationMatrix().rotateY(-t3));

  Pose3D T0to2 = ((T0to5 * T4to5Inv) * T3to4Inv) * T2to3Inv;

  float c0, s0, c1, s1, c2, s2;

  float x = 1 / std::sqrt(2.f);

  // Matrixzugriff: (Zeile, Spalte)
  float X1 = T0to2.rotation[0][0]; // = T0to2(1,1);
  // double X2 = T0to2(1,2);
  float X3 = T0to2.rotation[2][0]; // = T0to2(1,3);
  float X4 = T0to2.rotation[0][1]; // = T0to2(2,1);
  float X5 = T0to2.rotation[1][1]; // = T0to2(2,2);
  float X6 = T0to2.rotation[2][1]; // = T0to2(2,3);
  float X7 = T0to2.rotation[0][2]; // = T0to2(3,1);
  float X8 = T0to2.rotation[1][2]; // = T0to2(3,2);
  float X9 = T0to2.rotation[2][2]; // = T0to2(3,3);

  float X10 = sign * X6 + X9; // = c2(c1-s1)
  float X11 = sign * X4 + X7; // = s2(s1-c1)
  float X12 = X5 - sign * X8; // = c0(c1-s1)

  // X10 might be zero in the following cases:
  // 1.: c2 might be 0. This cannot be, because t2 is in [-100,25] and therefore can neither be -180 nor 180.
  // 2.: (c1-s1) might be 0. This can happen if t1 is 45 (or -135, but this is not possible).
  if (X10 == 0) {
    // ERROR HANDLING NEEDED!
    OUTPUT(idText, text, "something wrong in NaoKinematic");
    t0 = t1 = t2 = 0;
    result = false;
  } else // X10 != 0   => c2 != 0  && (c1-s1) != 0
  {
    t2 = std::atan(-X11 / X10);
    s2 = std::sin(t2);
    c2 = std::cos(t2);

    c0 = c2 * X1 + s2 * X3;
    s0 = -(c2 * X7 + s2 * X9) / x;
    t0 = std::atan2(s0, c0);

    if (c0 == 0) {
      c1 = X5 + X10 / (2 * c2); // c1 != 0, see above
      s1 = X5 - X10 / (2 * c2);
    } else if (s2 == 0) // && c0 != 0)
    {
      float X14 = X5 - sign * X6;  // = s1 + c0c1
      s1 = (X14 - X12) / (1 + c0); // c0 won't be -1, because t0 can neither be -180 nor 180
      c1 = (X14 + X12 / c0) / (1 + c0);
    } else // (c0 != 0) && (s0 != 0)
    {
      float X13 = -X7 / s2 - sign * X6 / c2 + x * s0 * s2 / c2 - x * s0 * c2 / s2; // = c0(c1+s1)
      c1 = (X12 + X13) / (2 * c0);                                                 // c0 != 0, because "else"...
      s1 = (-X12 + X13) / (2 * c0);
    }
    t1 = std::atan2(s1, c1);
  }

  // sign's are necessary because bredo-framework uses a different joint calibration than the rotation directions shown in
  // the above mentioned figure provided by aldebaran.
  jointRequest.jointAngles.angles[whichSideJoint0 + 0] = (float)t0;        // [-90 ;  0 ] = [...
  jointRequest.jointAngles.angles[whichSideJoint0 + 1] = (float)sign * t1; // [-100; 25 ] = [...
  jointRequest.jointAngles.angles[whichSideJoint0 + 2] = (float)t2;        // [-45 ; 25 ]
  jointRequest.jointAngles.angles[whichSideJoint0 + 3] = (float)t3;        // [ 0  ; 130]
  jointRequest.jointAngles.angles[whichSideJoint0 + 4] = (float)t4;        // [-75 ; 45 ]
  jointRequest.jointAngles.angles[whichSideJoint0 + 5] = (float)t5;        // [-25 ; 45 ]

  return result;
}

void NaoKinematic::update(KinematicOutput& kinematicOutput) {
#if 0
    MODIFY("NaoKinematic:rcxpKinematicRequest", rcxpKinematic);

    rcxpKinematic.calculated = false;
    DEBUG_RESPONSE("NaoKinematic:rcxpKinematic",
            Vector3<> leftFootRCXP(rcxpKinematic.kinematicRequest[0], rcxpKinematic.kinematicRequest[1], rcxpKinematic.kinematicRequest[2]);
            Vector3<> leftRotRCXP(rcxpKinematic.kinematicRequest[3], rcxpKinematic.kinematicRequest[4], rcxpKinematic.kinematicRequest[5]);
            Vector3<> rightFootRCXP(rcxpKinematic.kinematicRequest[6], rcxpKinematic.kinematicRequest[7], rcxpKinematic.kinematicRequest[8]);
            Vector3<> rightRotRCXP(rcxpKinematic.kinematicRequest[9], rcxpKinematic.kinematicRequest[10], rcxpKinematic.kinematicRequest[11]);
            calcLegJoints(JointData::LHipYawPitch, leftFootRCXP, leftRotRCXP, rcxpKinematic.jointRequest, theRobotDimensions);
            calcLegJoints(JointData::RHipYawPitch, rightFootRCXP, rightRotRCXP, rcxpKinematic.jointRequest, theRobotDimensions);

            rcxpKinematic.calculated = true;
            );
#endif

  Vector3<> leftFoot(theKinematicRequest.leftFoot[0], theKinematicRequest.leftFoot[1], theKinematicRequest.leftFoot[2]);
  Vector3<> rightFoot(theKinematicRequest.rightFoot[0], theKinematicRequest.rightFoot[1], theKinematicRequest.rightFoot[2]);
  Vector3<> leftFootRot(theKinematicRequest.leftFoot[3], theKinematicRequest.leftFoot[4], theKinematicRequest.leftFoot[5]);
  Vector3<> rightFootRot(
    theKinematicRequest.rightFoot[3], theKinematicRequest.rightFoot[4], theKinematicRequest.rightFoot[5]);

  // double mean=0;

  float distLeft = std::sqrt(sqr(leftFoot[0]) + sqr(leftFoot[1]) + sqr(leftFoot[2]));
  float distRight = std::sqrt(sqr(rightFoot[0]) + sqr(rightFoot[1]) + sqr(rightFoot[2]));

  switch (theKinematicRequest.kinematicType) {
  case KinematicRequest::feet:
    // Stand on the foot, which is nearer to the center
    // This is foot has more pressure when the robot moves slowly
    if (!useBHKinematics ||
        !InverseKinematic::calcLegJoints(
          Pose3D(RotationMatrix(leftFootRot.x, leftFootRot.y, leftFootRot.z), leftFoot + Vector3<>(0, 0, 40)),
          Pose3D(RotationMatrix(rightFootRot.x, rightFootRot.y, rightFootRot.z), rightFoot + Vector3<>(0, 0, 40)),
          kinematicOutput.jointAngles,
          theRobotDimensions)) {
      if (distLeft < distRight) {
        calcLegJoints(JointData::LHipYawPitch, leftFoot, leftFootRot, kinematicOutput, theRobotDimensions);
        calcLegJoints(JointData::RHipYawPitch,
                      rightFoot,
                      rightFootRot,
                      kinematicOutput.jointAngles.angles[JointData::LHipYawPitch],
                      kinematicOutput,
                      theRobotDimensions);
      } else {
        calcLegJoints(JointData::RHipYawPitch, rightFoot, rightFootRot, kinematicOutput, theRobotDimensions);
        calcLegJoints(JointData::LHipYawPitch,
                      leftFoot,
                      leftFootRot,
                      kinematicOutput.jointAngles.angles[JointData::RHipYawPitch],
                      kinematicOutput,
                      theRobotDimensions);
      }
    }

    break;

  case KinematicRequest::bodyAndLeftFoot:
    rightFoot = checkConstraints(leftFoot, leftFootRot.z, rightFoot, rightFootRot.z, false);
    if (!useBHKinematics ||
        !InverseKinematic::calcLegJoints(
          Pose3D(RotationMatrix(leftFootRot.x, leftFootRot.y, leftFootRot.z), leftFoot + Vector3<>(0, 0, 40)),
          Pose3D(RotationMatrix(rightFootRot.x, rightFootRot.y, rightFootRot.z), rightFoot + Vector3<>(0, 0, 40)),
          kinematicOutput.jointAngles,
          theRobotDimensions)) {
      calcLegJoints(JointData::LHipYawPitch, leftFoot, leftFootRot, kinematicOutput, theRobotDimensions);
      calcLegJoints(JointData::RHipYawPitch,
                    rightFoot,
                    rightFootRot,
                    kinematicOutput.jointAngles.angles[JointData::LHipYawPitch],
                    kinematicOutput,
                    theRobotDimensions);
    }
    // kinematicOutput.jointAngles.angles[JointData::LHipRoll]+=0.1;
    break;

  case KinematicRequest::bodyAndRightFoot:
    leftFoot = checkConstraints(leftFoot, leftFootRot.z, rightFoot, rightFootRot.z, true);
    if (!useBHKinematics ||
        !InverseKinematic::calcLegJoints(
          Pose3D(RotationMatrix(leftFootRot.x, leftFootRot.y, leftFootRot.z), leftFoot + Vector3<>(0, 0, 40)),
          Pose3D(RotationMatrix(rightFootRot.x, rightFootRot.y, rightFootRot.z), rightFoot + Vector3<>(0, 0, 40)),
          kinematicOutput.jointAngles,
          theRobotDimensions)) {
      calcLegJoints(JointData::RHipYawPitch, rightFoot, rightFootRot, kinematicOutput, theRobotDimensions);
      calcLegJoints(JointData::LHipYawPitch,
                    leftFoot,
                    leftFootRot,
                    kinematicOutput.jointAngles.angles[JointData::RHipYawPitch],
                    kinematicOutput,
                    theRobotDimensions);
    } else {
      // kinematicOutput.jointAngles.angles[JointData::RHipRoll]-=0.1;
      break;
    }
  default:
    break;
  }
#ifdef WALKING_SIMULATOR
  for (int i = (int)JointData::LHipYawPitch; i < 12; i++) {
    kinematicOutput.jointAngles.angles[i] += theKinematicRequest.offsets.angles[i];
  }
#endif
#ifdef TARGET_SIM
  // If this happens, go back on stack to ModuleManager and check
  // all Modules in "providers"
  for (int i = 0; i < JointData::numOfJoints; i++) {
    ASSERT(kinematicOutput.jointAngles.angles[i] == kinematicOutput.jointAngles.angles[i]);
  }
#endif

  kinematicOutput.jointAngles.angles[JointData::LShoulderPitch] = JointData::ignore;
  kinematicOutput.jointAngles.angles[JointData::LShoulderRoll] = JointData::ignore;
  kinematicOutput.jointAngles.angles[JointData::LElbowYaw] = JointData::ignore;
  kinematicOutput.jointAngles.angles[JointData::LElbowRoll] = JointData::ignore;

  kinematicOutput.jointAngles.angles[JointData::RShoulderPitch] = JointData::ignore;
  kinematicOutput.jointAngles.angles[JointData::RShoulderRoll] = JointData::ignore;
  kinematicOutput.jointAngles.angles[JointData::RElbowYaw] = JointData::ignore;
  kinematicOutput.jointAngles.angles[JointData::RElbowRoll] = JointData::ignore;
  kinematicOutput.jointAngles.angles[JointData::HeadYaw] = JointData::ignore;
  kinematicOutput.jointAngles.angles[JointData::HeadPitch] = JointData::ignore;

  if ((kinematicOutput.jointAngles.angles[JointData::LHipYawPitch] == JointData::ignore) ||
      (kinematicOutput.jointAngles.angles[JointData::LHipYawPitch] == JointData::off)) {
    kinematicOutput.jointAngles.angles[JointData::LHipYawPitch] =
      theKinematicRequest.offsets.angles[JointData::LHipYawPitch];
  } else {
    kinematicOutput.jointAngles.angles[JointData::LHipYawPitch] +=
      theKinematicRequest.offsets.angles[JointData::LHipYawPitch];
  }

  if ((kinematicOutput.jointAngles.angles[JointData::LHipRoll] == JointData::ignore) ||
      (kinematicOutput.jointAngles.angles[JointData::LHipRoll] == JointData::off)) {
    kinematicOutput.jointAngles.angles[JointData::LHipRoll] = theKinematicRequest.offsets.angles[JointData::LHipRoll];
  } else {
    kinematicOutput.jointAngles.angles[JointData::LHipRoll] += theKinematicRequest.offsets.angles[JointData::LHipRoll];
  }

  if ((kinematicOutput.jointAngles.angles[JointData::LHipPitch] == JointData::ignore) ||
      (kinematicOutput.jointAngles.angles[JointData::LHipPitch] == JointData::off)) {
    kinematicOutput.jointAngles.angles[JointData::LHipPitch] = theKinematicRequest.offsets.angles[JointData::LHipPitch];
  } else {
    kinematicOutput.jointAngles.angles[JointData::LHipPitch] += theKinematicRequest.offsets.angles[JointData::LHipPitch];
  }

  if ((kinematicOutput.jointAngles.angles[JointData::LKneePitch] == JointData::ignore) ||
      (kinematicOutput.jointAngles.angles[JointData::LKneePitch] == JointData::off)) {
    kinematicOutput.jointAngles.angles[JointData::LKneePitch] = theKinematicRequest.offsets.angles[JointData::LKneePitch];
  } else {
    kinematicOutput.jointAngles.angles[JointData::LKneePitch] += theKinematicRequest.offsets.angles[JointData::LKneePitch];
  }

  if ((kinematicOutput.jointAngles.angles[JointData::LAnklePitch] == JointData::ignore) ||
      (kinematicOutput.jointAngles.angles[JointData::LAnklePitch] == JointData::off)) {
    kinematicOutput.jointAngles.angles[JointData::LAnklePitch] = theKinematicRequest.offsets.angles[JointData::LAnklePitch];
  } else {
    kinematicOutput.jointAngles.angles[JointData::LAnklePitch] += theKinematicRequest.offsets.angles[JointData::LAnklePitch];
  }

  if ((kinematicOutput.jointAngles.angles[JointData::LAnkleRoll] == JointData::ignore) ||
      (kinematicOutput.jointAngles.angles[JointData::LAnkleRoll] == JointData::off)) {
    kinematicOutput.jointAngles.angles[JointData::LAnkleRoll] = theKinematicRequest.offsets.angles[JointData::LAnkleRoll];
  } else {
    kinematicOutput.jointAngles.angles[JointData::LAnkleRoll] += theKinematicRequest.offsets.angles[JointData::LAnkleRoll];
  }

  if ((kinematicOutput.jointAngles.angles[JointData::RHipYawPitch] == JointData::ignore) ||
      (kinematicOutput.jointAngles.angles[JointData::RHipYawPitch] == JointData::off)) {
    kinematicOutput.jointAngles.angles[JointData::RHipYawPitch] =
      theKinematicRequest.offsets.angles[JointData::RHipYawPitch];
  } else {
    kinematicOutput.jointAngles.angles[JointData::RHipYawPitch] +=
      theKinematicRequest.offsets.angles[JointData::RHipYawPitch];
  }

  if ((kinematicOutput.jointAngles.angles[JointData::RHipRoll] == JointData::ignore) ||
      (kinematicOutput.jointAngles.angles[JointData::RHipRoll] == JointData::off)) {
    kinematicOutput.jointAngles.angles[JointData::RHipRoll] = theKinematicRequest.offsets.angles[JointData::RHipRoll];
  } else {
    kinematicOutput.jointAngles.angles[JointData::RHipRoll] += theKinematicRequest.offsets.angles[JointData::RHipRoll];
  }

  if ((kinematicOutput.jointAngles.angles[JointData::RHipPitch] == JointData::ignore) ||
      (kinematicOutput.jointAngles.angles[JointData::RHipPitch] == JointData::off)) {
    kinematicOutput.jointAngles.angles[JointData::RHipPitch] = theKinematicRequest.offsets.angles[JointData::RHipPitch];
  } else {
    kinematicOutput.jointAngles.angles[JointData::RHipPitch] += theKinematicRequest.offsets.angles[JointData::RHipPitch];
  }

  if ((kinematicOutput.jointAngles.angles[JointData::RKneePitch] == JointData::ignore) ||
      (kinematicOutput.jointAngles.angles[JointData::RKneePitch] == JointData::off)) {
    kinematicOutput.jointAngles.angles[JointData::RKneePitch] = theKinematicRequest.offsets.angles[JointData::RKneePitch];
  } else {
    kinematicOutput.jointAngles.angles[JointData::RKneePitch] += theKinematicRequest.offsets.angles[JointData::RKneePitch];
  }

  if ((kinematicOutput.jointAngles.angles[JointData::RAnklePitch] == JointData::ignore) ||
      (kinematicOutput.jointAngles.angles[JointData::RAnklePitch] == JointData::off)) {
    kinematicOutput.jointAngles.angles[JointData::RAnklePitch] = theKinematicRequest.offsets.angles[JointData::RAnklePitch];
  } else {
    kinematicOutput.jointAngles.angles[JointData::RAnklePitch] += theKinematicRequest.offsets.angles[JointData::RAnklePitch];
  }

  if ((kinematicOutput.jointAngles.angles[JointData::RAnkleRoll] == JointData::ignore) ||
      (kinematicOutput.jointAngles.angles[JointData::RAnkleRoll] == JointData::off)) {
    kinematicOutput.jointAngles.angles[JointData::RAnkleRoll] = theKinematicRequest.offsets.angles[JointData::RAnkleRoll];
  } else {
    kinematicOutput.jointAngles.angles[JointData::RAnkleRoll] += theKinematicRequest.offsets.angles[JointData::RAnkleRoll];
  }

#ifdef WARN_ANGLES
  if (kinematicOutput.jointAngles.angles[JointData::LHipYawPitch] < -1.58 ||
      kinematicOutput.angles[JointData::LHipYawPitch] > 0.01)
    OUTPUT(
      idText, text, "Angle out of Range (LHipYawPitch): " << kinematicOutput.jointAngles.angles[JointData::LHipYawPitch]);
  if (kinematicOutput.jointAngles.angles[JointData::LHipRoll] < -0.79 || kinematicOutput.angles[JointData::LHipRoll] > 0.44)
    OUTPUT(idText, text, "Angle out of Range (LHipRoll): " << kinematicOutput.jointAngles.angles[JointData::LHipRoll]);
  if (kinematicOutput.jointAngles.angles[JointData::LHipPitch] < -1.75 ||
      kinematicOutput.angles[JointData::LHipPitch] > 0.44)
    OUTPUT(idText, text, "Angle out of Range (LHipPitch): " << kinematicOutput.jointAngles.angles[JointData::LHipPitch]);
  if (kinematicOutput.jointAngles.angles[JointData::LKneePitch] < -0.01 ||
      kinematicOutput.angles[JointData::LKneePitch] > 2.27)
    OUTPUT(idText, text, "Angle out of Range (lKneePitch): " << kinematicOutput.jointAngles.angles[JointData::LKneePitch]);
  if (kinematicOutput.jointAngles.angles[JointData::LAnklePitch] < -1.31 ||
      kinematicOutput.angles[JointData::LAnklePitch] > 0.79)
    OUTPUT(idText, text, "Angle out of Range (lAnklePitch): " << kinematicOutput.jointAngles.angles[JointData::LAnklePitch]);
  if (kinematicOutput.jointAngles.angles[JointData::LAnkleRoll] < -0.44 ||
      kinematicOutput.angles[JointData::LAnkleRoll] > 0.79)
    OUTPUT(idText, text, "Angle out of Range (lAnkleRoll): " << kinematicOutput.jointAngles.angles[JointData::LAnkleRoll]);
  if (kinematicOutput.jointAngles.angles[JointData::RHipYawPitch] < -1.58 ||
      kinematicOutput.angles[JointData::RHipYawPitch] > 0.01)
    OUTPUT(
      idText, text, "Angle out of Range (rHipYawPitch): " << kinematicOutput.jointAngles.angles[JointData::RHipYawPitch]);
  if (kinematicOutput.jointAngles.angles[JointData::RHipRoll] < -0.79 || kinematicOutput.angles[JointData::RHipRoll] > 0.44)
    OUTPUT(idText, text, "Angle out of Range (rHipRoll): " << kinematicOutput.jointAngles.angles[JointData::RHipRoll]);
  if (kinematicOutput.jointAngles.angles[JointData::RHipPitch] < -1.75 ||
      kinematicOutput.angles[JointData::RHipPitch] > 0.44)
    OUTPUT(idText, text, "Angle out of Range (rHipPitch): " << kinematicOutput.jointAngles.angles[JointData::RHipPitch]);
  if (kinematicOutput.jointAngles.angles[JointData::RKneePitch] < -0.01 ||
      kinematicOutput.angles[JointData::RKneePitch] > 2.27)
    OUTPUT(idText, text, "Angle out of Range (rKneePitch): " << kinematicOutput.jointAngles.angles[JointData::RKneePitch]);
  if (kinematicOutput.jointAngles.angles[JointData::RAnklePitch] < -1.31 ||
      kinematicOutput.angles[JointData::RAnklePitch] > 0.79)
    OUTPUT(idText, text, "Angle out of Range (rAnklePitch): " << kinematicOutput.jointAngles.angles[JointData::RAnklePitch]);
  if (kinematicOutput.jointAngles.angles[JointData::RAnkleRoll] < -0.44 ||
      kinematicOutput.angles[JointData::RAnkleRoll] > 0.79)
    OUTPUT(idText, text, "Angle out of Range (rAnkleRoll): " << kinematicOutput.jointAngles.angles[JointData::RAnkleRoll]);
#endif
  /*LOG("ExternalSimulator.csv", "LHipRoll", kinematicOutput.jointAngles.angles[JointData::LHipRoll]);
  LOG("ExternalSimulator.csv", "LHipPitch", kinematicOutput.jointAngles.angles[JointData::LHipPitch]);
  LOG("ExternalSimulator.csv", "lKneePitch", kinematicOutput.jointAngles.angles[JointData::LKneePitch]);
  LOG("ExternalSimulator.csv", "lAnklePitch", kinematicOutput.jointAngles.angles[JointData::LAnklePitch]);
  LOG("ExternalSimulator.csv", "lAnkleRoll", kinematicOutput.jointAngles.angles[JointData::LAnkleRoll]);

  LOG("ExternalSimulator.csv", "rHipRoll", kinematicOutput.jointAngles.angles[JointData::RHipRoll]);
  LOG("ExternalSimulator.csv", "rHipPitch", kinematicOutput.jointAngles.angles[JointData::RHipPitch]);
  LOG("ExternalSimulator.csv", "rKneePitch", kinematicOutput.jointAngles.angles[JointData::RKneePitch]);
  LOG("ExternalSimulator.csv", "rAnklePitch", kinematicOutput.jointAngles.angles[JointData::RAnklePitch]);
  LOG("ExternalSimulator.csv", "rAnkleRoll", kinematicOutput.jointAngles.angles[JointData::RAnkleRoll]); */
}

MAKE_MODULE(NaoKinematic, dortmundWalkingEngine)

/**
 * @file KickEngineData.h
 *
 * This file declares a module that creates the kicking motions.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#pragma once

#include "KickEngineParameters.h"
#include "Core/System/BHAssert.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Core/RingBufferWithSum.h"
#include "Core/Math/Matrix.h"
#include "Core/Math/Pose2D.h"

#include <vector>

class KickEngineData {
private:
  bool toLeftSupport = false;
  bool formMode = false;
  bool limbOff[Phase::numOfLimbs];
  bool startComp = false;

  int phaseNumber = 0;
  int phaseNumberPrevious = phaseNumber;
  int motionID = -1;

  float phase = 0.f;
  float cycletime = 0.012f;
  float lastRatio = 0.f;

  unsigned int timestamp = 0;
  unsigned int timestampStable = 0;
  unsigned int timestampKickStarted = 0;
  int timeSinceTimestamp = 0;
  int timeSinceKickStarted = 0;

  Vector2<> bodyAngle;
  Vector2<> balanceSum;
  Vector2<> gyro;
  Vector2<> lastGyroLeft;
  Vector2<> lastGyroRight;
  Vector2<> gyroErrorLeft;
  Vector2<> gyroErrorRight;
  Vector2<> lastBody;
  Vector2<> bodyError;
  Pose2D lastOdometry;
  bool lElbowFront = false, rElbowFront = false;

  // Parameter for P, I and D for gyro PID Control
  Vector2<> gyroP = Vector2<>(3.f, -2.5f);
  Vector2<> gyroI;
  Vector2<> gyroD = Vector2<>(0.03f, 0.01f);

  Vector2<> head;

  Vector3<> origins[Phase::numOfLimbs];

  Vector3<> torsoRot;

  Vector3<> lastCom;
  Vector3<> ref;
  Vector3<> actualDiff;
  Vector2<> balanceAdjustment;
  RobotModel comRobotModel;

  JointRequest lastBalancedJointRequest;
  JointRequest compenJoints;

  bool wasActive = false;
  bool willBeLeft = false;
  float lastXDif = 0.f;
  float lastZDif = 0.f;
  bool fastKickEndAdjusted = false;
  float adjustedXValue = 0.f;
  float adjustedZValue = 0.f;

public:
  RobotModel robotModel;
  BallModel theBallModel;
  KickEngineParameters currentParameters;

  omniKickParameters omniKickParams;
  float kickAngle;
  float kickPower;

  float balanceAngleX;
  float balanceAngleY;

  bool validBallPosition;
  bool controllPointExtendedFlags[Phase::numOfLimbs] = {0};
  std::vector<float> previewRefPointLookAhead = {50};
  float timeStep;
  std::vector<float> A1;
  std::vector<float> A2;
  std::vector<float> A3;
  std::vector<float> B;

  std::vector<int> remainingDurationToPhase;
  Vector2<> relBallPosition;
  std::vector<float> yStateLeft = {50.0f, 0.0f, 0.0f};
  std::vector<float> yStateRight = {50.0f, 0.0f, 0.0f};
  KickRequest currentKickRequest;
  bool internalIsLeavingPossible = false;
  Vector3<> positions[Phase::numOfLimbs];
  bool getMotionIDByName(const KickRequest& kr, const std::vector<KickEngineParameters>& params);
  void calculateOrigins(const KickRequest& kr,
                        const JointData& ja,
                        const TorsoMatrix& to,
                        const RobotDimensions& theRobotDimensions);
  bool checkPhaseTime(const FrameInfo& frame, const JointData& ja, const TorsoMatrix& torsoMatrix);

  float linearInterpolation(const float& value, const float& goal, const float& maxSpeed);
  void balanceCOM(JointRequest& joints, const RobotDimensions& rd, const MassCalibration& mc);

  bool
  calcJoints(JointRequest& jointRequest, const RobotDimensions& rd, const DamageConfiguration& theDamageConfigurationBody);
  void calcOdometryOffset(KickEngineOutput& output, const RobotModel& theRobotModel);
  void calcLegJoints(const JointData::Joint& joint,
                     JointRequest& jointRequest,
                     const RobotDimensions& theRobotDimensions,
                     const DamageConfiguration& theDamageConfiguration);
  void simpleCalcArmJoints(const JointData::Joint& joint,
                           JointRequest& jointRequest,
                           const RobotDimensions& theRobotDimensions,
                           const Vector3<>& armPos,
                           const Vector3<>& handRotAng);

  void mirrorIfNecessary(JointRequest& joints);
  void addGyroBalance(JointRequest& jointRequest,
                      const JointCalibration& jointCalibration,
                      const InertiaSensorData& id,
                      const float& ratio);
  void addDynPoint(const DynPoint& dynPoint, const TorsoMatrix& torsoMatrix);
  void ModifyData(const KickRequest& br, JointRequest& kickEngineOutput, std::vector<KickEngineParameters>& params);
  void calcPhaseState();
  bool kickPhaseReached();
  void calcPositions(const TorsoMatrix& torsoMatrix, KickRequest::KickMotionID& kickMotionType);

  void updateBallPosition(const BallModel& theBallModel);
  bool decideKickLeg(float kickAngle, const BallModel& theBallModel);
  void extendControlPoints(const int& phaseNumber);
  void extendControlPointsSwingBackPhase(const int& phaseNumber, const int& limb);
  void extendControlPointsKickPhase(const int& phaseNumber, const int& limb);
  void updatePreviewRefPoints();
  void updateRemainingDuration();
  int checkPhaseNumberLimit(int phaseNumber);
  void stateEvolution(std::vector<float>& state);

  void setExecutedKickRequest(KickRequest& br);
  void initData(const FrameInfo& frame,
                KickRequest& kr,
                std::vector<KickEngineParameters>& params,
                const JointData& ja,
                const TorsoMatrix& torsoMatrix,
                JointRequest& jointRequest,
                const RobotDimensions& rd,
                const MassCalibration& mc,
                const DamageConfiguration& theDamageConfigurationBody,
                const BallModel& theBallModel,
                const omniKickParameters& theNewKickParameters);
  void setEngineActivation(const float& ratio);
  bool activateNewMotion(const KickRequest& br, const bool& isLeavingPossible);
  bool sitOutTransitionDisturbance(bool& compensate,
                                   bool& compensated,
                                   const InertiaSensorData& id,
                                   KickEngineOutput& kickEngineOutput,
                                   const JointRequest& theJointRequest,
                                   const FrameInfo& frame);
  void BOOST(JointRequest& jointRequest, int boostPhase);
  bool adjustFastKickHack(const TorsoMatrix& torsoMatrix);
  void addJointDataDebuggingPlot(const JointData& JD, const JointRequest& JDReq);
  KickEngineData() {
    for (int i = 0; i < Phase::numOfLimbs; i++) {
      limbOff[i] = false;
    }
  }
};

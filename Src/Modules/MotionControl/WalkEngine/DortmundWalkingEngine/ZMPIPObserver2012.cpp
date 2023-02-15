/**
 * @file ZMPIPObserver2012.cpp
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#include "ZMPIPObserver2012.h"
#include "Core/Debugging/Modify.h"

void ZMPIPObserver2012::update(ObservedError& observedError) {
  static int delayCounter = 0;

  if (wasOn && !theTargetCoM.isRunning) {
    wasOn = false;
    delayBuffer.clear();
    coMDelayBuffer.clear();
  }
  if (!wasOn && theTargetCoM.isRunning) {
    wasOn = true;
  }

  Vector2<> xOffset(theWalkingEngineParams.comOffsets.xFixed, 0);
  xOffset.rotate(theWalkingInfo.robotPosition.rotation);

  Vector2<> target(theTargetCoM.state_x[2] + xOffset.x, theTargetCoM.state_y[2] + xOffset.y);
  Vector2<> targetCoM(theTargetCoM.state_x[0] + xOffset.x, theTargetCoM.state_y[0] + xOffset.y);

  delayBuffer.push_front(Vector2f(target.x, target.y));
  coMDelayBuffer.push_front(Vector2f(targetCoM.x, targetCoM.y));

  Vector2<> realZMP(theZMPModel.ZMP_WCS.x, theZMPModel.ZMP_WCS.y);
  Vector2<> realCoM(theActualCoM.x, theActualCoM.y);
  Pose2D robotPosition(theWalkingInfo.robotPosition.rotation,
                       Vector2<>(theWalkingInfo.robotPosition.translation.x, theWalkingInfo.robotPosition.translation.y));

  Vector2<> ZMPdiff, CoMdiff;
  if (delayBuffer.size() > theWalkingEngineParams.sensorControl.sensorDelay) {
    ZMPdiff = realZMP - Vector2<>((delayBuffer[theWalkingEngineParams.sensorControl.sensorDelay]).x,
                                  (delayBuffer[theWalkingEngineParams.sensorControl.sensorDelay]).y);
    CoMdiff = realCoM - Vector2<>((coMDelayBuffer[theWalkingEngineParams.sensorControl.halSensorDelay]).x,
                                  (coMDelayBuffer[theWalkingEngineParams.sensorControl.halSensorDelay]).y);
  } else {
    ZMPdiff = Vector2<>(0, 0);
    CoMdiff = Vector2<>(0, 0);
  }

#if 1
  static bool sensorOn = false;

  if ((thePatternGenRequest.newState == PatternGenRequest::walking) && isStable) {
    sensorOn = true;
  }

  if (!(thePatternGenRequest.newState == PatternGenRequest::walking)) {
    sensorOn = false;
  }
#else
  static bool sensorOn = true;
#endif

  if (sensorOn && localSensorScale < 1) {
    localSensorScale += 1.0f / theWalkingEngineParams.walkTransition.zmpSmoothPhase;
  }

  if (!sensorOn) {
    delayCounter++;
  } else {
    delayCounter = 0;
  }

  if (delayCounter > theControllerParams.N && localSensorScale > 0) {
    localSensorScale -= 1.0f / theWalkingEngineParams.walkTransition.zmpSmoothPhase;
  }

  ZMPdiff.rotate(-robotPosition.rotation);
  ZMPdiff *= (theWalkingEngineParams.sensorControl.sensorControlRatio[1] * localSensorScale);
  ZMPdiff.rotate(robotPosition.rotation);

  CoMdiff.rotate(-robotPosition.rotation);
  CoMdiff *= (localSensorScale * theWalkingEngineParams.sensorControl.sensorControlRatio[0]);
  CoMdiff.rotate(robotPosition.rotation);

  MODIFY("module:ZMPIPObserver:CoMdiff", CoMdiff);
  MODIFY("module:ZMPIPObserver:ZMPdiff", ZMPdiff);

  observedError.CoM_WCS[0][0] = theControllerParams.L * Vector2f(CoMdiff.x, 0);
  observedError.ZMP_WCS[0][0] = theControllerParams.L * Vector2f(0, ZMPdiff.x);
  observedError.CoM_WCS[0][1] = theControllerParams.L * Vector2f(CoMdiff.y, 0);
  observedError.ZMP_WCS[0][1] = theControllerParams.L * Vector2f(0, ZMPdiff.y);

  PLOT("module:ZMPIPObserver2012:Delayed_CoM.x", coMDelayBuffer[theWalkingEngineParams.sensorControl.halSensorDelay].x);
  PLOT("module:ZMPIPObserver2012:Delayed_CoM.y", coMDelayBuffer[theWalkingEngineParams.sensorControl.halSensorDelay].y);
  PLOT("module:ZMPIPObserver2012:Delayed_ZMP.x", delayBuffer[theWalkingEngineParams.sensorControl.sensorDelay].x);
  PLOT("module:ZMPIPObserver2012:Delayed_ZMP.y", delayBuffer[theWalkingEngineParams.sensorControl.sensorDelay].y);
  PLOT("module:ZMPIPObserver2012:ZMPDiff.x", ZMPdiff.x);
  PLOT("module:ZMPIPObserver2012:ZMPDiff.y", ZMPdiff.y);

  /*if (theTargetCoM.isRunning)
  {
    LOG("WalkingEngine", "real ZMP x", realZMP.x);
    LOG("WalkingEngine", "real ZMP y", realZMP.y);
  }*/
}

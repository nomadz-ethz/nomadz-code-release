#include "ZMPController.h"
#include "Core/Streams/InStreams.h"

void ZMPController::update(const Leg::Side supportSide) {
  MODIFY("parameters:ZMPControllerParams", static_cast<ZMPControllerParams&>(*this));
  transitionPhaseDuration =
    transitionPhaseBaseDuration +
    thePlannedSteps.currentPlannedSpeed.translation.x * transitionPhaseDurationIncreaseFactor.translation.x +
    thePlannedSteps.currentPlannedSpeed.translation.y * transitionPhaseDurationIncreaseFactor.translation.y +
    thePlannedSteps.currentPlannedSpeed.rotation * transitionPhaseDurationIncreaseFactor.rotation;
  calculationHorizon = std::ceil((thePlannedSteps.upcomingSteps[0].stepDuration +
                                  thePlannedSteps.upcomingSteps[1].stepDuration + transitionPhaseDuration) /
                                 Constants::motionCycleTime);
  generateZMPRef(supportSide);
  previewController();
}

void ZMPController::reset() {}

void ZMPController::calcDefaultOriginRef() {}

void ZMPController::updateInitialStates() {
  initialStateX(0) = theWalkGeneratorData.lastOriginOffset.x / mmPerM;
  initialStateX(1) = theWalkGeneratorData.currentSpeed.translation.x / mmPerM;
  initialStateX(2) = 0.f;
  initialStateY(0) = theWalkGeneratorData.lastOriginOffset.y / mmPerM;
  initialStateY(1) = theWalkGeneratorData.currentSpeed.translation.y / mmPerM;
  initialStateY(2) = 0.f;
}

void ZMPController::previewController() {
  updateInitialStates();
  Eigen::Vector3f state_x = initialStateX;
  Eigen::Vector3f state_y = initialStateY;
  comRef.clear();
  zmp.clear();
  for (int i = 0; i < calculationHorizon; ++i) {
    float zmpX = C_d * state_x;
    float zmpY = C_d * state_y;
    float errorX = zmpRef[0].x / mmPerM - zmpX;
    float errorY = zmpRef[0].y / mmPerM - zmpY;

    float previewX = 0;
    float previewY = 0;
    for (int k = i; k < i + previewHorizon; ++k) {
      previewX += previewGain[k - i] * zmpRef[k].x / mmPerM;
      previewY += previewGain[k - i] * zmpRef[k].y / mmPerM;
    }
    float u_x = -G_i * errorX - G_x.dot(state_x) - previewX;
    float u_y = -G_i * errorY - G_x.dot(state_y) - previewY;
    state_x = A_d * state_x + B_d * u_x;
    state_y = A_d * state_y + B_d * u_y;
    comRef.push_back(Vector2<>(state_x(0), state_y(0)) * mmPerM);
    zmp.push_back(Vector2<>(C_d * state_x, C_d * state_y) * mmPerM);
  }
}

void ZMPController::generateZMPRef(const Leg::Side supportSide) {
  int nextStepIndex = 0;
  float lastTimeStamp = 0.f;

  Pose2D nextSupportFoot = thePlannedSteps.measuredStep[supportSide];
  Pose2D prevSupportFoot = thePlannedSteps.measuredStep[(supportSide == Leg::left) ? Leg::right : Leg::left];
  Pose2D upcomingStep;
  zmpRefBasePoints.clear();
  originBasePoints.clear();
  originBasePoints.push_back(PointTime());
  while (lastTimeStamp < (calculationHorizon + previewHorizon) * Constants::motionCycleTime) {
    if (nextStepIndex > thePlannedSteps.upcomingSteps.size() - 1) {
      break;
    }
    upcomingStep = thePlannedSteps.upcomingSteps[nextStepIndex].legPose;
    Pose2D currentOrigin = calcOriginFromSteps(prevSupportFoot, nextSupportFoot);
    Pose2D nextOrigin = calcOriginFromSteps(nextSupportFoot, upcomingStep);
    float stepDuration = thePlannedSteps.upcomingSteps[nextStepIndex].stepDuration;
    Vector2<> zmpSegmentStart = projZMPSpeedRef2SupportPolygon(nextSupportFoot, currentOrigin.translation);
    Vector2<> zmpSegmentEnd = projZMPSpeedRef2SupportPolygon(nextSupportFoot, nextOrigin.translation);
    zmpSegmentEnd =
      (zmpSegmentEnd - zmpSegmentStart) * stepDuration / (stepDuration + transitionPhaseDuration) + zmpSegmentStart;
    zmpRefBasePoints.push_back(PointTime(zmpSegmentStart, lastTimeStamp));
    zmpRefBasePoints.push_back(PointTime(zmpSegmentEnd, lastTimeStamp + stepDuration));
    originBasePoints.push_back(PointTime(nextOrigin, lastTimeStamp + stepDuration + transitionPhaseDuration));
    currentOrigin = nextOrigin;

    nextStepIndex++;
    lastTimeStamp += thePlannedSteps.upcomingSteps[nextStepIndex].stepDuration + transitionPhaseDuration;
    prevSupportFoot = nextSupportFoot;
    nextSupportFoot = upcomingStep;
  }

  int nextZMPRefIndex = 0;
  int nextOriginRefIndex = 0;
  lastTimeStamp = 0;
  PointTime currentZMPRefPoint = zmpRefBasePoints[nextZMPRefIndex];
  PointTime nextZMPRefPoint = zmpRefBasePoints[nextZMPRefIndex];

  PointTime currentOriginPoint = originBasePoints[nextZMPRefIndex];
  PointTime nextOriginPoint = originBasePoints[nextZMPRefIndex + 1];
  zmpRef.clear();
  originRef.clear();
  for (int i = 0; i < calculationHorizon + previewHorizon; ++i) {
    if (i * Constants::motionCycleTime >= nextZMPRefPoint.time) {
      if (nextZMPRefIndex >= zmpRefBasePoints.size() - 1) {
        break;
      }
      lastTimeStamp = nextZMPRefPoint.time;
      nextZMPRefIndex++;
      currentZMPRefPoint = nextZMPRefPoint;
      nextZMPRefPoint = zmpRefBasePoints[nextZMPRefIndex];
    }
    if (i * Constants::motionCycleTime >= nextOriginPoint.time) {
      nextOriginRefIndex++;
      currentOriginPoint = nextOriginPoint;
      nextOriginPoint = originBasePoints[nextOriginRefIndex];
    }
    Vector2<> zmpRefPoint_i = nextZMPRefPoint.point.translation * (i * Constants::motionCycleTime - lastTimeStamp) /
                                (nextZMPRefPoint.time - lastTimeStamp) +
                              currentZMPRefPoint.point.translation * (1 - (i * Constants::motionCycleTime - lastTimeStamp) /
                                                                            (nextZMPRefPoint.time - lastTimeStamp));

    Pose2D originRefPoint_i =
      nextOriginPoint.point
        .scale(std::min(1.f,
                        (i * Constants::motionCycleTime - currentOriginPoint.time) /
                          (nextOriginPoint.time - transitionPhaseDuration - currentOriginPoint.time)))
        .elementwiseAdd(currentOriginPoint.point.scale(
          (1 - std::min(1.f,
                        (i * Constants::motionCycleTime - currentOriginPoint.time) /
                          (nextOriginPoint.time - transitionPhaseDuration - currentOriginPoint.time)))));

    zmpRef.push_back(zmpRefPoint_i);
    originRef.push_back(originRefPoint_i);
  }
}

Vector2<> ZMPController::projZMPSpeedRef2SupportPolygon(const Pose2D& upcomingSupportFoot, const Vector2<>& point) {
  Rangef supportFootZMPRange(-theRobotDimensions.footBack + zmpRefEdgeMargin,
                             theRobotDimensions.footFront - zmpRefEdgeMargin);

  Vector2<> directionPoint = upcomingSupportFoot * Vector2<>(1.f, 0.f);
  Vector2<> foot2Point = point - upcomingSupportFoot.translation;
  Vector2<> foot2front = directionPoint - upcomingSupportFoot.translation;
  const float projectScaler = foot2front.dot(foot2Point);
  Vector2<> projPoint = foot2front * supportFootZMPRange.limit(projectScaler);
  return projPoint + upcomingSupportFoot.translation;
}

Pose2D ZMPController::calcOriginFromSteps(const Pose2D& prevStep, const Pose2D& nextStep) {
  return prevStep.elementwiseAdd(nextStep).scale(0.5f) + Pose2D(Vector2<>(torsoOffset, 0.f));
}
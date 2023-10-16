/**
 * @file ZMPController.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <Eigen/Dense>
#include "Core/Math/Vector.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/PlannedSteps.h"
#include "Representations/MotionControl/WalkRequest.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/RobotModel.h"
#include "Core/Debugging/Modify.h"

STREAMABLE(ZMPControllerParams,
           {
             ,
             (int)previewHorizon,
             (float)errorGain,
             (std::vector<float>)stateGain,
             (std::vector<float>)previewGain,
             (float)transitionPhaseBaseDuration,
             (Pose2D)transitionPhaseDurationIncreaseFactor,
             (float)zmpRefEdgeMargin,
           });

struct PointTime {
  Pose2D point;
  float time;
  PointTime() {
    point = Pose2D();
    time = 0;
  };
  PointTime(Vector2<> p, float t) {
    point = Pose2D(p);
    time = t;
  };
  PointTime(Pose2D p, float t) {
    point = p;
    time = t;
  };
};

class ZMPController : public ZMPControllerParams {
public:
  ZMPController(const PlannedSteps& thePlannedSteps,
                const WalkGeneratorData& theWalkGeneratorData,
                const RobotDimensions& theRobotDimensions,
                const RobotModel& theRobotModel)
      : thePlannedSteps(thePlannedSteps), theWalkGeneratorData(theWalkGeneratorData), theRobotDimensions(theRobotDimensions),
        theRobotModel(theRobotModel) {
    InMapFile stream("ZMPControllerParams.cfg");
    if (stream.exists()) {
      stream >> static_cast<ZMPControllerParams&>(*this);
    }
    zmpRef.reserve(previewHorizon);
    comRef.reserve(previewHorizon);
  };

  void reset();
  void update(const Leg::Side supportSide);
  void generateZMPRef(const Leg::Side supportSide);
  void previewController();
  void applyCorrection();

private:
  const float mmPerM = 1000.f;
  const PlannedSteps& thePlannedSteps;
  const WalkGeneratorData& theWalkGeneratorData;
  const RobotModel& theRobotModel;
  const RobotDimensions& theRobotDimensions;
  float torsoOffset = 12.f;
  int calculationHorizon;
  void updateInitialStates();
  void calcDefaultOriginRef();
  Vector2<> projZMPSpeedRef2SupportPolygon(const Pose2D& upcomingSupportFoot, const Vector2<>& point);
  Vector2<> calcOriginFromStepsIndex(int& prevStepIndex, int& nextStepIndex);
  Pose2D calcOriginFromSteps(const Pose2D& prevStep, const Pose2D& nextStep);

  const Eigen::Matrix3f A_d = (Eigen::Matrix3f() << 1.f, 0.012f, 0.000072f, 0.f, 1.f, 0.012f, 0.f, 0.f, 1.f).finished();
  const Eigen::Matrix<float, 3, 1> B_d = Eigen::Matrix<float, 3, 1>(0.000000288f, 0.000072f, 0.012f);
  const Eigen::Matrix<float, 1, 3> C_d = Eigen::Matrix<float, 1, 3>(1.f, 0.f, -0.0268f);

  const Eigen::Vector3f G_x = Eigen::Vector3f(7472.1482473, 1466.3439973, 45.6710739);
  const float G_i = 235.6633;
  Eigen::Vector3f initialStateX;
  Eigen::Vector3f initialStateY;

  Eigen::VectorXf zmpRefX = Eigen::VectorXf::Zero(previewHorizon);
  Eigen::VectorXf zmpRefY = Eigen::VectorXf::Zero(previewHorizon);

  Eigen::VectorXf comRefX;
  Eigen::VectorXf comRefY;

public:
  std::vector<PointTime> zmpRefBasePoints;
  std::vector<Vector2<>> zmpRef;
  std::vector<Vector2<>> zmp;
  std::vector<Vector2<>> comRef;
  std::vector<PointTime> originBasePoints;
  std::vector<Pose2D> originRef;
  float transitionPhaseDuration;
};
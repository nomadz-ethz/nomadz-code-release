/**
 * @file FLIPMController.cpp
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#include "FLIPMController.h"
#include "Tools/DortmundWalkingEngine/CSVLogger.h"

FLIPMController::FLIPMController() {
  InMapFile xCfg("FLIPMContX.cfg");
  xCfg >> paramsFLIPMContX;

  InMapFile yCfg("FLIPMContY.cfg");
  yCfg >> paramsFLIPMContY;

  reset();
}

void FLIPMController::reset() {
  // std::cout << "FLIPM reset!" << std::endl;
  x[0][0] = Vector6f();
  x[0][1] = Vector6f();
  v = Vector2f();

  x_RCS[0][0] = Vector6f();
  x_RCS[0][1] = Vector6f();
  v_RCS = Vector2f();

  pRef.clear();
  pRef_RCS.clear();

  kElementEmpty = true;
  kElementRCSEmpty = true;

  isRunning = false;
}

// ZMP FLIPMController::getReferenceZMP() {
//   if (isRunning) {
//     return *kElement;
//   } else
//     return ZMP();
// }

// Rensen: Removed due to unknown attribute warning
//#ifdef TARGET_ROBOT
//__attribute__((optimize("unroll-loops")))
//#endif
void FLIPMController::executeController(Dimension d, const Matrix<1, static_N, float>& refZMP) {
  Matrix<6, 1, float> err = theObservedFLIPMError.ObservedError.c[0][d]; //+ theReferenceModificator.handledErr[d];
  /*Eigen::Matrix< float, 6, 1> offsetFromParameters = Eigen::Matrix<float, 6, 1>::Zero();
  offsetFromParameters(0, 0) = theWalkingEngineParams.comOffsets.xFixed;
  float yOffset = theWalkingEngineParams.comOffsets.ySpeedDependent[theFootpositions.speed.y < 0] * 1000.0f *
  std::fabs(theFootpositions.speed.y * 1000.0f) / theWalkingEngineParams.speedLimits.y;
  offsetFromParameters(1, 0) = theWalkingEngineParams.comOffsets.yFixed + yOffset;*/

  if (d == X) {
    float u1 = paramsFLIPMContX.Gi * v[d];
    float u2 = (paramsFLIPMContX.Gx * x[0][d])[0][0];
    float u3 = (refZMP * paramsFLIPMContX.Gd)[0][0];
    float u = -u1 - u2 - u3;
    Matrix<6, 1, float> x1 = paramsFLIPMContX.A * x[0][d];
    Matrix<6, 1, float> x2 = paramsFLIPMContX.b * u;
    x[0][d] = x1 + x2 + err; // +offsetFromParameters;
    float v1 = (paramsFLIPMContX.c * x[0][d])[0][0];
    float v2 = (*kElement)[d];
    v[d] += v1 - v2;
  } else {
    float u1 = paramsFLIPMContY.Gi * v[d];
    float u2 = (paramsFLIPMContY.Gx * x[0][d])[0][0];
    float u3 = (refZMP * paramsFLIPMContY.Gd)[0][0];
    float u = -u1 - u2 - u3;
    Matrix<6, 1, float> x1 = paramsFLIPMContY.A * x[0][d];
    Matrix<6, 1, float> x2 = paramsFLIPMContY.b * u;
    x[0][d] = x1 + x2 + err; // +offsetFromParameters;
    float v1 = (paramsFLIPMContY.c * x[0][d])[0][0];
    float v2 = (*kElement)[d];
    v[d] += v1 - v2;
  }
}

void FLIPMController::executeRCSController(Dimension d, const Matrix<1, static_N, float>& refZMP) {
  Matrix<6, 1, float> err = theObservedFLIPMError.ObservedError.c[0][d]; //+ theReferenceModificator.handledErr[d];

  if (d == X) {
    float u1 = paramsFLIPMContX.Gi * v_RCS[d];
    float u2 = (paramsFLIPMContX.Gx * x_RCS[0][d])[0][0];
    float u3 = (refZMP * paramsFLIPMContX.Gd)[0][0];
    float u = -u1 - u2 - u3;
    Matrix<6, 1, float> x1 = paramsFLIPMContX.A * x_RCS[0][d];
    Matrix<6, 1, float> x2 = paramsFLIPMContX.b * u;
    x_RCS[0][d] = x1 + x2 + err;
    float v1 = (paramsFLIPMContX.c * x_RCS[0][d])[0][0];
    float v2 = (*kElement_RCS)[d];
    v_RCS[d] += v1 - v2;
  } else {
    float u1 = paramsFLIPMContY.Gi * v_RCS[d];
    float u2 = (paramsFLIPMContY.Gx * x_RCS[0][d])[0][0];
    float u3 = (refZMP * paramsFLIPMContY.Gd)[0][0];
    float u = -u1 - u2 - u3;
    Matrix<6, 1, float> x1 = paramsFLIPMContY.A * x_RCS[0][d];
    Matrix<6, 1, float> x2 = paramsFLIPMContY.b * u;
    x_RCS[0][d] = x1 + x2 + err;
    float v1 = (paramsFLIPMContY.c * x_RCS[0][d])[0][0];
    float v2 = (*kElement_RCS)[d];
    v_RCS[d] += v1 - v2;
  }
}
Point FLIPMController::controllerStep() {
  PLOT("module:FLIPMController:refZMP.x", kElement->x);
  PLOT("module:FLIPMController:refZMP.y", kElement->y);
  PLOT("module:FLIPMController:refZMP_RCS.x", kElement_RCS->x);
  PLOT("module:FLIPMController:refZMP_RCS.y", kElement_RCS->y);

  Matrix<2, 1, Matrix<1, static_N, float>> refZMP;

  ZMPList::iterator _pRef;
  _pRef = kElement;

// Rensen: added because of warning above
#ifdef TARGET_ROBOT
#pragma unroll
#endif
  for (int i = 0; i < static_N; i++) {
    refZMP[0][0][i][0] = _pRef->x;
    refZMP[0][1][i][0] = _pRef->y;

    _pRef++;
  }
  executeController(X, refZMP[0][0]);
  executeController(Y, refZMP[0][1]);

  kElement++;

  Matrix<2, 1, Matrix<1, static_N, float>> refZMP_RCS;
  ZMPList::iterator _pRef_RCS;
  _pRef_RCS = kElement_RCS;

// Rensen: added because of warning above
#ifdef TARGET_ROBOT
#pragma unroll
#endif
  for (int i = 0; i < static_N; i++) {
    refZMP_RCS[0][0][i][0] = _pRef_RCS->x;
    refZMP_RCS[0][1][i][0] = _pRef_RCS->y;

    _pRef_RCS++;
  }
  executeRCSController(X, refZMP_RCS[0][0]);
  executeRCSController(Y, refZMP_RCS[0][1]);

  Pose2D robotPosition(theWalkingInfo.robotPosition.rotation,
                       Vector2<>(theWalkingInfo.robotPosition.translation.x, theWalkingInfo.robotPosition.translation.y));
  Point targetCOM(x_RCS[0][0][0][3], x_RCS[0][1][0][3]);

  // float direction = kElement_RCS->direction;
  // targetCOM.rotate2D(direction);

  kElement_RCS++;

  PLOT("module:FLIPMController:targetCoM_RCS.x", targetCOM.x);
  PLOT("module:FLIPMController:targetCoM_RCS.y", targetCOM.y);
  PLOT("module:FLIPMController:targetCoM.x", x[0][0][0][3]);
  PLOT("module:FLIPMController:targetCoM.y", x[0][1][0][3]);
  PLOT("module:FLIPMController:targetCoMDiff.x", x[0][0][0][3] - targetCOM.x);
  PLOT("module:FLIPMController:targetCoMDiff.y", x[0][1][0][3] - targetCOM.y);

  if (theFLIPMObserverParams.useRCS) {
    return Point(targetCOM.x, targetCOM.y, paramsFLIPMContX.z_h);
  } else {
    return Point(x[0][0][0][3], x[0][1][0][3], paramsFLIPMContX.z_h);
  }
}

/*double FLIPMController::normalizeAngle(double angle)
{
    double twoPi = 2.0 * pi;
    double a = angle - twoPi * floor(angle / twoPi);
    if (a > pi) {
        a -= 2 * pi;
    }
    else if (a < -pi) {
        a += 2 * pi;
    }

    return a;
}*/

void FLIPMController::addFootsteps(const Footposition& fp) {
  Footposition* newpos = new Footposition;
  *newpos = fp;

  footPositions.push_back(newpos);
}

void FLIPMController::update(TargetCoM& targetCoM) {
  DECLARE_PLOT("module:FLIPMController:refZMP.x");
  DECLARE_PLOT("module:FLIPMController:refZMP.y");
  DECLARE_PLOT("module:FLIPMController:refZMP_RCS.x");
  DECLARE_PLOT("module:FLIPMController:refZMP_RCS.y");
  DECLARE_PLOT("module:FLIPMController:calcZMP.x");
  DECLARE_PLOT("module:FLIPMController:calcZMP.y");
  DECLARE_PLOT("module:FLIPMController:calcAcc.x");
  DECLARE_PLOT("module:FLIPMController:calcAcc.y");
  DECLARE_PLOT("module:FLIPMController:x1");
  DECLARE_PLOT("module:FLIPMController:x2");
  DECLARE_PLOT("module:FLIPMController:y1");
  DECLARE_PLOT("module:FLIPMController:y2");
  DECLARE_PLOT("module:FLIPMController:angleX");
  DECLARE_PLOT("module:FLIPMController:angleY");
  DECLARE_PLOT("module:FLIPMController:targetCoM.x");
  DECLARE_PLOT("module:FLIPMController:targetCoM.y");

  DECLARE_PLOT("module:FLIPMController:calcZMP_RCS.x");
  DECLARE_PLOT("module:FLIPMController:calcZMP_RCS.y");
  DECLARE_PLOT("module:FLIPMController:x1_RCS");
  DECLARE_PLOT("module:FLIPMController:x2_RCS");
  DECLARE_PLOT("module:FLIPMController:y1_RCS");
  DECLARE_PLOT("module:FLIPMController:y2_RCS");
  DECLARE_PLOT("module:FLIPMController:targetCoM_RCS.x");
  DECLARE_PLOT("module:FLIPMController:targetCoM_RCS.y");
  DECLARE_PLOT("module:FLIPMController:targetCoMDiff.x");
  DECLARE_PLOT("module:FLIPMController:targetCoMDiff.y");

  // paramsFLIPMContX.handle();
  // paramsFLIPMContY.handle();

  if (!isRunning && theRefZMP.running) /* if the controller isn't running, but we have several new reference ZMP's*/
  {
    Start();
    isRunning = true;
  }

  for (int i = 0; i < theFootSteps.getNumOfSteps(); i++) {
    addFootsteps(theFootSteps.getStep(i));
  }

  for (int i = 0; i < theRefZMP.numOfZMP_RCS; i++) /* Fetch the new reference ZMP's and push them in our ZMPList*/
  {
    addRefZMP_RCS(theRefZMP.getZMP_RCS(i));
  }
  for (int i = 0; i < theRefZMP.numOfZMP; i++) /* Fetch the new reference ZMP's and push them in our ZMPList*/
  {
    addRefZMP(theRefZMP.getZMP(i));
  }

  targetCoM.x = targetCoM.y = 0; /* reset the target COM*/
  for (int i = 0; i < 6; i++) {
    targetCoM.state_x[i] = 0.0;
    targetCoM.state_y[i] = 0.0;
  }

  if (isRunning && !theRefZMP.running) /* if the controller is running, but we have no new reference ZMP's*/
  {
    End();
    isRunning = false;
  }

  MARK("FLIPMController", "x1p"); /*x1 Position*/
  MARK("FLIPMController", "x1s"); /*x1 Speed*/
  MARK("FLIPMController", "x1a"); /*x1 Acceleration*/
  MARK("FLIPMController", "x2p"); /*x2 Position*/
  MARK("FLIPMController", "x2s"); /*x2 Speed*/
  MARK("FLIPMController", "x2a"); /*x2 Acceleration*/
  MARK("FLIPMController", "y1p"); /*y1 Position*/
  MARK("FLIPMController", "y1s"); /*y1 Speed*/
  MARK("FLIPMController", "y1a"); /*y1 Acceleration*/
  MARK("FLIPMController", "y2p"); /*y2 Position*/
  MARK("FLIPMController", "y2s"); /*y2 Speed*/
  MARK("FLIPMController", "y2a"); /*y2 Acceleration*/
  MARK("FLIPMController", "calcZMPx");
  MARK("FLIPMController", "calcZMPy");
  MARK("FLIPMController", "pRefX");
  MARK("FLIPMController", "pRefY");

  LOG("FLIPMController", "CoPx", theZMPModel.ZMP_WCS.x());
  LOG("FLIPMController", "CoPy", theZMPModel.ZMP_WCS.y());
  LOG("FLIPMController", "angleX", theInertiaSensorData.angle.x());
  LOG("FLIPMController", "angleY", theInertiaSensorData.angle.y());

  if (isRunning) {
    ASSERT(pRef.size() > 0);
    ASSERT(pRef_RCS.size() > 0);

    // Elemente, die vom ZMPGenerator modifiziert wurden und im selben Frame rausgeschickt wurden,
    // werden hier eingehangen und dann nochmal modififiert.
    (Point&)targetCoM = controllerStep(); /* Set the targetCoM*/

    for (int i = 0; i < 6; i++) {
      targetCoM.state_x[i] = x[0][0][0][i];
      targetCoM.state_y[i] = x[0][1][0][i];
    }

    ASSERT(x[0][0] == x[0][0]);

    Shrink();

    PLOT("module:FLIPMController:x1", x[0][0][0][0]); /*x1 Position*/
    PLOT("module:FLIPMController:x2", x[0][0][0][3]); /*x2 Position*/
    PLOT("module:FLIPMController:y1", x[0][1][0][0]); /*y1 Position*/
    PLOT("module:FLIPMController:y2", x[0][1][0][3]); /*y2 Position*/
    PLOT("module:FLIPMController:calcZMP.y", (paramsFLIPMContX.c * x[0][1])[0][0]);
    PLOT("module:FLIPMController:calcZMP.x", (paramsFLIPMContY.c * x[0][0])[0][0]);

    PLOT("module:FLIPMController:x1_RCS", x_RCS[0][0][0][0]); /*x1 Position*/
    PLOT("module:FLIPMController:x2_RCS", x_RCS[0][0][0][3]); /*x2 Position*/
    PLOT("module:FLIPMController:y1_RCS", x_RCS[0][1][0][0]); /*y1 Position*/
    PLOT("module:FLIPMController:y2_RCS", x_RCS[0][1][0][3]); /*y2 Position*/
    PLOT("module:FLIPMController:calcZMP_RCS.x", (paramsFLIPMContX.c * x_RCS[0][0])[0][0]);
    PLOT("module:FLIPMController:calcZMP_RCS.y", (paramsFLIPMContY.c * x_RCS[0][1])[0][0]);

    PLOT("module:FLIPMController:calcAcc.y", x[0][1][0][2]);
    PLOT("module:FLIPMController:calcAcc.x", x[0][0][0][2]);
    PLOT("module:FLIPMController:angleX", theInertiaSensorData.angle.x);
    PLOT("module:FLIPMController:angleY", theInertiaSensorData.angle.y);

    LOG("FLIPMController", "x1p", x[0][0][0][0]); /*x1 Position*/
    LOG("FLIPMController", "x1s", x[0][0][0][1]); /*x1 Speed*/
    LOG("FLIPMController", "x1a", x[0][0][0][2]); /*x1 Acceleration*/
    LOG("FLIPMController", "x2p", x[0][0][0][3]); /*x2 Position*/
    LOG("FLIPMController", "x2s", x[0][0][0][4]); /*x2 Speed*/
    LOG("FLIPMController", "x2a", x[0][0][0][5]); /*x2 Acceleration*/
    LOG("FLIPMController", "y1p", x[0][1][0][0]); /*y1 Position*/
    LOG("FLIPMController", "y1s", x[0][1][0][1]); /*y1 Speed*/
    LOG("FLIPMController", "y1a", x[0][1][0][2]); /*y1 Acceleration*/
    LOG("FLIPMController", "y2p", x[0][1][0][3]); /*y2 Position*/
    LOG("FLIPMController", "y2s", x[0][1][0][4]); /*y2 Speed*/
    LOG("FLIPMController", "y2a", x[0][1][0][5]); /*y2 Acceleration*/
    LOG("FLIPMController", "calcZMPx", paramsFLIPMContX.c * x[0][0]);
    LOG("FLIPMController", "calcZMPy", paramsFLIPMContY.c * x[0][1]);
    LOG("FLIPMController", "pRefX", kElement->x);
    LOG("FLIPMController", "pRefY", kElement->y);
  }

  targetCoM.isRunning = isRunning;
}

void FLIPMController::Shrink() {
  while (pRef.begin() != kElement) {
    pRef.pop_front();
  }

  while (pRef_RCS.begin() != kElement_RCS) {
    pRef_RCS.pop_front();

    Footposition* footPos = footPositions.front();
    footPositions.pop_front();
    delete footPos;
  }
}

void FLIPMController::addRefZMP(ZMP zmp) {
  pRef.push_back(zmp);

  if (kElementEmpty) {
    kElement = pRef.begin();
    kElementEmpty = false;
  }
}

void FLIPMController::addRefZMP_RCS(ZMP zmp) {
  pRef_RCS.push_back(zmp);

  if (kElementRCSEmpty) {
    kElement_RCS = pRef_RCS.begin();
    kElementRCSEmpty = false;
  }
}

MAKE_MODULE(FLIPMController, dortmundWalkingEngine)

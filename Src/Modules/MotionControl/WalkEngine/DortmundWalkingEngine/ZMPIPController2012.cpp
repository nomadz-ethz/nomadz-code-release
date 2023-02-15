/**
 * @file ZMPIPController2012.cpp
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#include "ZMPIPController2012.h"
#define LOGGING
#ifndef WALKING_SIMULATOR
#include "Tools/DortmundWalkingEngine/CSVLogger.h"
#else
#include "csvlogger.h"
#endif

ZMPIPController2012::ZMPIPController2012(const RefZMP& theRefZMP,
                                         const WalkingEngineParams& theWalkingEngineParams,
                                         const ControllerParams& theControllerParams,
                                         const ObservedError& theObservedError,
                                         const ReferenceModificator& theReferenceModificator)
    : theRefZMP(theRefZMP),
      // theWalkingEngineParams(theWalkingEngineParams),
      theControllerParams(theControllerParams), theObservedError(theObservedError),
      theReferenceModificator(theReferenceModificator), positionDelayBuffer(Point()) {
  reset();
}

void ZMPIPController2012::Shrink() {
  while (pRef.begin() != kElement) {
    pRef.pop_front();
  }
}

void ZMPIPController2012::reset() {
  obs.c[0] = Vector3f(0, 0, 0);
  obs.c[1] = Vector3f(0, 0, 0);
  v = Vector2f(0, 0);
  pRef.clear();

  kElementEmpty = true;

  isRunning = false;
  positionDelayBuffer.clear();
}

ZMP ZMPIPController2012::getReferenceZMP() {
  if (isRunning) {
    return *kElement;
  } else {
    return ZMP();
  }
}

// Rensen: Removed due to unknown attribute warning
//#ifdef TARGET_ROBOT
//__attribute__((optimize("unroll-loops")))
//#endif
void ZMPIPController2012::executeController(Dimension d, const Matrix<1, ControllerParams::N, float>& refZMP) {
  Vector3f err = theObservedError.ZMP_WCS.c[0][d] + theReferenceModificator.handledErr.c[0][d];

  float u = (refZMP * theControllerParams.Gd)[0][0];
  float cont = -theControllerParams.Gi * v[d] - (theControllerParams.Gx * obs.c[0][d])[0] - u;
  obs.c[d] = theControllerParams.A0 * obs.c[0][d] + err + theControllerParams.b0 * cont;
  v[d] += (theControllerParams.c0 * obs.c[0][d])[0] - (*kElement)[d];
}

// Rensen: Removed due to unknown attribute warning
//#ifdef TARGET_ROBOT
//__attribute__((optimize("unroll-loops")))
//#endif
Point ZMPIPController2012::controllerStep() {
  ZMPList::iterator _pRef = kElement;
  Matrix<2, 1, Matrix<1, ControllerParams::N, float>> refZMP;

  PLOT("module:ZMPIPController2012:refZMP.x", _pRef->x);
  PLOT("module:ZMPIPController2012:refZMP.y", _pRef->y);
  PLOT("module:ZMPIPController2012:calcZMP.x", obs.c[0][0][2]);
  PLOT("module:ZMPIPController2012:calcZMP.y", obs.c[0][1][2]);

// Rensen: Added because of warning above
#ifdef TARGET_ROBOT
#pragma unroll
#endif
  for (int i = 0; i < ControllerParams::N; i++) {
    refZMP[0][0][i] = _pRef->x;
    refZMP[1][0][i] = _pRef->y;
    _pRef++;
  }

  executeController(X, refZMP.c[0][0][0]);
  executeController(Y, refZMP.c[0][1][0]);

  kElement++;

  return Point(obs[0][0][0], obs[0][1][0], theControllerParams.z_h);
}

void ZMPIPController2012::addRefZMP(ZMP zmp) {
  pRef.push_back(zmp);

  if (kElementEmpty) {
    kElement = pRef.begin();
    kElementEmpty = false;
  }
}

void ZMPIPController2012::getObservations(Vector3f& x, Vector3f& y) {
  x = obs.c[0][0];
  y = obs.c[0][1];
}

void ZMPIPController2012::updateKinematicRequest(TargetCoM& targetCoM) {

  DECLARE_PLOT("module:ZMPIPController2012:refZMP.x");
  DECLARE_PLOT("module:ZMPIPController2012:refZMP.y");
  DECLARE_PLOT("module:ZMPIPController2012:calcZMP.x");
  DECLARE_PLOT("module:ZMPIPController2012:calcZMP.y");
  DECLARE_PLOT("module:ZMPIPController2012:targetCoM.x");
  DECLARE_PLOT("module:ZMPIPController2012:targetCoM.y");
  DECLARE_PLOT("module:ZMPIPController2012:calcZMP.y");

  for (int i = 0; i < theRefZMP.numOfZMP; i++) {
    addRefZMP(theRefZMP.getZMP(i));
  }

  targetCoM.x = targetCoM.y = 0;

  if (!isRunning && theRefZMP.running) {
    Start();
    isRunning = true;
  }

  if (isRunning && !theRefZMP.running) {
    End();
    isRunning = false;
  }
  if (isRunning) {
    ASSERT(pRef.size() > 0);

    // Elemente, die vom ZMPGenerator modifiziert wurden und im selben Frame rausgeschickt wurden,
    // werden hier eingehangen und dann nochmal modififiert.

    (Point&)targetCoM = controllerStep();
    Shrink();
    /*LOG("WalkingEngine", "calc ZMP x", *obs.x[2]);
    LOG("WalkingEngine", "calc ZMP y", *obs.y[2]);
    LOG("WalkingEngine", "ref ZMP x", kElement->x);
    LOG("WalkingEngine", "ref ZMP y", kElement->y);
    LOG("WalkingEngine", "Target CoM x", targetCoM.x);
    LOG("WalkingEngine", "Target CoM y", targetCoM.y);
    LOG("WalkingEngine", "Target CoM z", targetCoM.z);*/

    for (ZMPList::iterator zmp = pRef.begin(); zmp != pRef.end(); zmp++) {
      for (int dim = 0; dim < 2; dim++) {
        if (theReferenceModificator.aTime[dim].startZMP != -1 &&
            (int)(zmp->timestamp) >= theReferenceModificator.aTime[dim].startZMP) {
          *zmp += ZMP(theReferenceModificator[dim].x, theReferenceModificator[dim].y);
        }
      }
    }
  }
  targetCoM.isRunning = isRunning;

  PLOT("module:ZMPIPController2012:targetCoM.x", targetCoM.x);
  PLOT("module:ZMPIPController2012:targetCoM.y", targetCoM.y);

  ASSERT(targetCoM.x == targetCoM.x && targetCoM.y == targetCoM.y);
}

/**
 * @file FLIPMController.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original authors are <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a> and <a
 * href="mailto:arne.moos@tu-dortmund.de> Arne Moos</a>
 */

#pragma once
/* tells the RingBuffer to check the boundaries */
#define LIMIT_CHECK

#include <list>
#include <stdio.h>
#include "Core/System/File.h"
#include "Core/Streams/OutStreams.h"
#include "Core/Streams/InStreams.h"
#include "Tools/DortmundWalkingEngine/StepData.h"
#include "Tools/DortmundWalkingEngine/Point.h"
#include "Core/Streams/AutoStreamable.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Representations/MotionControl/RefZMP.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/ReferenceModificator.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/MotionControl/Footpositions.h"
#include <Eigen/Eigen>
#include "Core/Module/Module.h"
#include "Representations/Sensing/ZMPModel.h"
#include "Representations/MotionControl/ObservedFLIPMError.h"
#include "Modules/MotionControl/WalkEngine/DortmundWalkingEngine/FLIPMObserver.h"

// const int static_N = 50; /**< Length of the preview phase */

using Matrix1x6f = Matrix<1, 6, float>;
using Matrix1x6d = Matrix<1, 6, double>;
using Vector50d = Matrix<50, 1, double>;
using Vector50f = Matrix<50, 1, float>;
using Matrix6x3d = Matrix<6, 3, double>;
using Matrix6x3f = Matrix<6, 3, float>;
using Matrix6f = Matrix<6, 6, float>;

MODULE(FLIPMController)
REQUIRES(WalkingEngineParams)
REQUIRES(ObservedFLIPMError)
REQUIRES(FLIPMObserverParams)
REQUIRES(RefZMP)
REQUIRES(InertiaSensorData)
REQUIRES(ZMPModel)
REQUIRES(FootSteps)
REQUIRES(Footpositions)
USES(WalkingInfo)
PROVIDES_WITH_MODIFY(TargetCoM)
END_MODULE

class FLIPMController : public FLIPMControllerBase {

  STREAMABLE(FLIPMContX,
             {
               ,
               (float)m,
               (float)M,
               (float)g,
               (float)z_h,
               (float)dt,
               (float)D,
               (float)E,
               (float)Qe,
               (float)Qx,
               (float)R,
               (int)N,
               (Matrix6f)A,
               (Vector6f)b,
               (Matrix1x6f)c,
               (float)Gi,
               (Matrix1x6f)Gx,
               (Vector50f)Gd,
             });

  STREAMABLE(FLIPMContY,
             {
               ,
               (float)m,
               (float)M,
               (float)g,
               (float)z_h,
               (float)dt,
               (float)D,
               (float)E,
               (float)Qe,
               (float)Qx,
               (float)R,
               (int)N,
               (Matrix6f)A,
               (Vector6f)b,
               (Matrix1x6f)c,
               (float)Gi,
               (Matrix1x6f)Gx,
               (Vector50f)Gd,
             });

  FLIPMContX paramsFLIPMContX;
  FLIPMContY paramsFLIPMContY;

public:
  /** Constructor with all needed source data structures.
   * @param theRefZMP The desired ZMP.
   * @param theWalkingEngineParams Walking Engine Parameters.
   */
  FLIPMController();

  ~FLIPMController() {} /**< Destructor */

  /** Start the controller. */
  void Start() { reset(); }

  /** Tells the controller to stop moving after the last added step. */
  void End() { reset(); }

  ZMP getReferenceZMP();

  /**
   * Calculate one target position of the CoM.
   * @param targetCoM The representation to fill with the position.
   */
  void update(TargetCoM& targetCoM);
  // void update(TargetCoMFLIPM & theTargetCoMFLIPM) { update((TargetCoM&)theTargetCoMFLIPM); };

private:
  typedef std::list<ZMP> ZMPList;
  /** Current element in the list. */
  ZMPList::iterator kElement;
  /** List of reference ZMP. */
  ZMPList pRef;

  /** Current element in the list. */
  ZMPList::iterator kElement_RCS;
  /** List of reference ZMP. */
  ZMPList pRef_RCS;

  Matrix<2, 1, Vector6f> x; /**< Observer state vector*/
  Vector2f v;               /**< Controller internal value. */

  Matrix<2, 1, Vector6f> x_RCS; /**< Observer state vector*/
  Vector2f v_RCS;               /**< Controller internal value. */

  bool isRunning,     /**< Is the controller running? */
    kElementEmpty,    /**< Is there a valid current ZMPList element? */
    kElementRCSEmpty; /**< Is there a valid current ZMPList element? */

  typedef std::list<Footposition*> FootList;
  FootList footPositions;                    /**< List of foot steps. */
  void addFootsteps(const Footposition& fp); /**< Add a foot position to the footPositions list. */

  void reset();                /**< Resets the controller. */
  void Shrink();               /** Deletes no more needed elements in lists. */
  void addRefZMP(ZMP zmp);     /** Add a ZMP position to the list. */
  void addRefZMP_RCS(ZMP zmp); /** Add a ZMP position to the list. */
  Point controllerStep();      /** Calculate one step of the system. */
  // double normalizeAngle(double angle);
  void executeController(Dimension d, const Matrix<1, static_N, float>& refZMP);
  void executeRCSController(Dimension d, const Matrix<1, static_N, float>& refZMP);
};

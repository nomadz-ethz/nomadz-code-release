/**
 * @file ZMPIPController2012.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once
/* tells the RingBuffer to check the boundaries */
#define LIMIT_CHECK

#include <list>
#include <stdio.h>
#include "Tools/DortmundWalkingEngine/StepData.h"
#include "Tools/DortmundWalkingEngine/Point.h"
#include "Representations/MotionControl/ObservedError.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Representations/MotionControl/RefZMP.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Core/RingBufferWithSumNew.h"
#include "Representations/MotionControl/ReferenceModificator.h"
#include "Tools/DortmundWalkingEngine/Filters/FIRFilter.h"
#include "Core/Module/Module.h"

#define MAX_SENSOR_DELAY 100

/**
 * @class ZMPIPController2012
 * Controller translation the desired ZMP position to a target CoM position. See
 *
 * Stefan Czarnetzki, S�ren Kerner, Oliver Urbann,
 * Observer-based dynamic walking control for biped robots,
 * Robotics and Autonomous Systems, Volume 57, Issue 8, Humanoid Soccer Robots, 31 July 2009, Pages 839-845, ISSN 0921-8890
 * doi:10.1016/j.robot.2009.03.007. http://linkinghub.elsevier.com/retrieve/pii/S0921889009000608.
 *
 */
class ZMPIPController2012 {

public:
  /** Constructor with all needed source data structures.
   * @param theRefZMP The desired ZMP.
   * @param theWalkingEngineParams Walking Engine Parameters.
   * @param thePatternGenRequest The request received by the PatternGenerator.
   * @param theFallDownState Information about the current state. If the robt has fallen down stop walking engine.
   * @param theControllerParams Some controller parameters used here.
   * @param theZMPModel The measured ZMP.
   * @param theWalkingInfo Informations about the walk from the last frame.
   */
  ZMPIPController2012(const RefZMP& theRefZMP,
                      const WalkingEngineParams& theWalkingEngineParams,
                      const ControllerParams& theControllerParams,
                      const ObservedError& theObservedError,
                      const ReferenceModificator& theReferenceModificator);

  ~ZMPIPController2012() {} /**< Destructor */

  /** Start the controller. */
  void Start() {}

  /** Tells the controller to stop moving after the last added step. */
  void End() { reset(); }

  /**
   * Returns the current observation
   * @param x Obersavtion vector for x axis (position, speed, ZMP)
   * @param y Obersavtion vector for y axis (position, speed, ZMP)
   */
  void getObservations(Vector3f& x, Vector3f& y);

  ZMP getReferenceZMP();

  /**
   * Calculate one target position of the CoM.
   * @param targetCoM The representation to fill with the position.
   */
  void updateKinematicRequest(TargetCoM& targetCoM);

private:
  const RefZMP& theRefZMP; /**< Set by constructor. */
                           // const WalkingEngineParams & theWalkingEngineParams;	/**< Set by constructor. */ unused
  const ControllerParams& theControllerParams; /**< Set by constructor. */
  const ObservedError& theObservedError;
  const ReferenceModificator& theReferenceModificator;

  typedef std::list<ZMP> ZMPList;
  /** Current element in the list. */
  ZMPList::iterator kElement;
  /** List of reference ZMP. */
  ZMPList pRef;

  RingBufferWithSumNew<Point, MAX_SENSOR_DELAY> positionDelayBuffer;

  Matrix<2, 2, Vector3f> obs; /**< Observer state vector*/
  // Vector2_D<> cont;             			/**< Controller internal value. */
  Vector2f v; /**< Controller internal value. */

  bool isRunning,  /**< Is the controller running? */
    kElementEmpty; /**< Is there a valid current ZMPList element? */

  void reset();            /**< Resets the controller. */
  void Shrink();           /** Deletes no more needed elements in lists. */
  void addRefZMP(ZMP zmp); /** Add a ZMP position to the list. */
  Point controllerStep();  /** Calculate one step of the system. */
  void executeController(Dimension d, const Matrix<1, ControllerParams::N, float>& refZMP);
};

/**
 * @file RequestTranslator.h
 *
 * Translates the MotionRequest for the WalkingEngine (some clipping and path-calculations)
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original authors are <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a> and  <a
 * href="mailto:stefan.czarnetzki@uni-dortmund.de">Stefan Czarnetzki</a>
 */
#pragma once

#include <list>
#include <algorithm>

#include <Eigen/Eigen>
#include "Tools/DortmundWalkingEngine/Point.h"

#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Core/Module/Module.h"
#include "Tools/DortmundWalkingEngine/Filters/FastFilter.h"
#include "Core/RingBuffer.h"
#include "Core/RingBufferWithSumNew.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/BehaviorControl/GoalSymbols.h"
#include "Core/Module/ModuleManager.h"
#include "Modules/MotionControl/WalkEngine/DortmundWalkingEngine/FLIPMObserver.h"

#define MAX_ACC_WINDOW 100

class RequestTranslator {
  STREAMABLE(walkingDecelerate,
             {
               ,
               (float)angle_border,
               (float)angleSpeedchange,
               (bool)disableDynamicSpeed,
               (float)fPos,
               (float)fNeg,
               (float)var_border,
               (unsigned int)standTime,
               (int)totalInitTime,
             });

  STREAMABLE(GoToRequest,
             {
               ,
               (float)kpx,
               (float)kpy,
               (float)kpr,
               (float)minSpeedForUsingSpeedSum,
               (float)maxSpeedSum,
             });

  walkingDecelerate paramswalkingDecelerate;
  GoToRequest paramsGoToRequest;

  friend class RequestTranslatorModule;

public:
  RequestTranslator(const MotionSelection& theMotionSelection,
                    const WalkingInfo& theWalkingInfo,
                    const FallDownState& theFallDownState,
                    const InertiaSensorData& theInertiaSensorData,
                    const BallModelAfterPreview& theBallModelAfterPreview,
                    const SpeedInfo& theSpeedInfo,
                    // const SpeedRequest &theSpeedRequest,
                    const ArmMovement& ArmMovement,
                    const GoalSymbols& theGoalSymbols);
  ~RequestTranslator(void);

  void updatePatternGenRequest(PatternGenRequest& patternGenRequest);
  void updateWalkingEngineParams(WalkingEngineParams& walkingEngineParams);
  void updateFLIPMObserverParams(FLIPMObserverParams& flipmObserverParams);

private:
  const MotionSelection& theMotionSelection;
  const WalkingInfo& theWalkingInfo;
  const FallDownState& theFallDownState;
  const InertiaSensorData& theInertiaSensorData;
  const BallModelAfterPreview& theBallModelAfterPreview;
  const SpeedInfo& theSpeedInfo;
  // const SpeedRequest &theSpeedRequest;
  const ArmMovement& theArmMovement;
  const GoalSymbols& theGoalSymbols;

  ModuleManager::Configuration config;
  FLIPMObserverParams observerParams;
  WalkingEngineParams walkParams;
  ControllerParams contParams;
  PatternGenRequest::State getNewState(float x, float y, float r);
  FastFilter<float> filter[3];
  PatternGenRequest::State currentState;
  int accCounter[3];
  int lastN{};
  bool walkStarted{}, firstStep{};
  float localSensorScale{};

  unsigned int timer{};
  bool deceleratedByMax{};
  bool deceleratedByInstability{};

  // general goto stuff
  Pose2D gotoPointController(const Pose2D& target, const Pose2D& max);
  Pose2D gotoPointController(const Vector2<>& target, const Pose2D& max);

  void clipping(Pose2D& speed);
  Pose2D gotoBallAndStand();
  void reset();
  Pose2D gotoBallAndKick();

  float tempMaxSpeed{}; // dynamicMaximumSpeed
  float theFactor{};    // factor by which the tempMaxSpeed ist modified

  /* 'timer' */
  int angle_cooldown{};
  int initTime{};
  int tempMaxSpeedInc{};
  int tempMaxSpeedDec{};

  /* status bools */
  bool angleTooHigh{};
  bool angleTriggered{};
  bool isInitialized{};

  /* Ringbuffer */
  RingBufferWithSumNew<float, 100> accX_data;
  RingBufferWithSumNew<float, 5> accX_var_buffer;
  // RingBufferWithSum<float,6000> speeds;

  /* Stuff for acceleration*/
  Point accelerate(Point p);
  typedef std::list<std::pair<int, Point>> AccList;
  AccList accBufferX; // FIXME: Value is used uninitialized
  AccList accBufferY; // FIXME: Value is used uninitialized
  AccList accBufferR; // FIXME: Value is used uninitialized
  float getLimitFac(Point& v, float maxAcc, int accDelay, int axis, AccList& accBuffer);
  float getSpdMin(int axis, AccList& accBuffer);
};

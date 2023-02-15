/**
 * @file RequestTranslatorModule.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#pragma once

#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Core/Module/Module.h"
#include "RequestTranslator.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Modules/MotionControl/WalkEngine/DortmundWalkingEngine/FLIPMObserver.h"
#include "Representations/BehaviorControl/GoalSymbols.h"

MODULE(RequestTranslatorModule)
USES(WalkingInfo)
REQUIRES(MotionSelection)
REQUIRES(FallDownState)
// REQUIRES(SpeedRequest)
REQUIRES(InertiaSensorData)
REQUIRES(BallModelAfterPreview)
USES(ArmMovement)
USES(SpeedInfo)
REQUIRES(GoalSymbols)
PROVIDES_WITH_MODIFY_AND_DRAW(PatternGenRequest)
PROVIDES_WITH_MODIFY(WalkingEngineParams)
PROVIDES_WITH_MODIFY(FLIPMObserverParams)
END_MODULE

class RequestTranslatorModule : public RequestTranslatorModuleBase {
public:
  RequestTranslatorModule();

protected:
  MotionSelection filteredMotionSelection;
  RequestTranslator translator;

  void update(PatternGenRequest& patternGenRequest);
  void update(WalkingEngineParams& walkingEngineParams);
  void update(FLIPMObserverParams& flipmObserverParams);

private:
  void save(WalkingEngineParams& params, std::string path);
  void save(FLIPMObserverParams& flipmObserverParams, std::string path);
  WalkingInfo theWalkingInfoDummy;
};

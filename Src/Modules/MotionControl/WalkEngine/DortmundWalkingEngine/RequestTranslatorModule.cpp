/**
 * @file RequestTranslatorModule.cpp
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#include "RequestTranslatorModule.h"
#include "Core/Settings.h"
#include "Core/System/File.h"
#include "Core/Streams/InOut.h"
#include "Core/Streams/OutStreams.h"

using namespace std;

RequestTranslatorModule::RequestTranslatorModule()
    : filteredMotionSelection(), translator(filteredMotionSelection,
                                            theWalkingInfoDummy,
                                            theFallDownState,
                                            theInertiaSensorData,
                                            theBallModelAfterPreview,
                                            theSpeedInfo,
                                            // theSpeedRequest,
                                            theArmMovement,
                                            theGoalSymbols) {}

void RequestTranslatorModule::save(WalkingEngineParams& params, string path) {
  OutMapFile stream(path);
  if (stream.exists()) {
    stream << params;
    OUTPUT(idText, text, string("Saved ") + path);
  } else {
    OUTPUT(idText, text, string("Failed to save ") + path);
  }
}

void RequestTranslatorModule::save(FLIPMObserverParams& flipmObserverParams, std::string path) {
  OutMapFile stream(path);
  if (stream.exists()) {
    stream << flipmObserverParams;
    OUTPUT(idText, text, string("Saved ") + path);
  } else {
    OUTPUT(idText, text, string("Failed to save ") + path);
  }
}

void RequestTranslatorModule::update(PatternGenRequest& patternGenRequest) {
  filteredMotionSelection = theMotionSelection;
  if (theMotionSelection.targetMotion != MotionRequest::walk || theMotionSelection.ratios[MotionRequest::walk] < 1.f) {
    filteredMotionSelection.walkRequest.speed.rotation = 0.f;
    filteredMotionSelection.walkRequest.speed.translation.x = 0.f;
    filteredMotionSelection.walkRequest.speed.translation.y = 0.f;
    filteredMotionSelection.walkRequest.mode = WalkRequest::speedMode;
  }
  translator.updatePatternGenRequest(patternGenRequest);
}

void RequestTranslatorModule::update(WalkingEngineParams& walkingEngineParams) {
  DEBUG_RESPONSE_ONCE("module:RequestTranslator:WalkingEngineParams:save_default",
                      save(walkingEngineParams, File::getBHDir() + string("/Config/Robots/Default/walkingParams.cfg")););
  DEBUG_RESPONSE_ONCE(
    "module:RequestTranslator:WalkingEngineParams:save_robot",
    save(walkingEngineParams,
         File::getBHDir() + string("/Config/Robots/") + Global::getSettings().robot + string("/walkingParams.cfg")););

  DEBUG_RESPONSE_ONCE(
    "module:RequestTranslator:WalkingEngineParamsFLIPM:save_default",
    save(walkingEngineParams, File::getBHDir() + string("/Config/Robots/Default/walkingParamsFLIPM.cfg")););
  DEBUG_RESPONSE_ONCE(
    "module:RequestTranslator:WalkingEngineParamsFLIPM:save_robot",
    save(walkingEngineParams,
         File::getBHDir() + string("/Config/Robots/") + Global::getSettings().robot + string("/walkingParamsFLIPM.cfg")););

  translator.updateWalkingEngineParams(walkingEngineParams);
}

void RequestTranslatorModule::update(FLIPMObserverParams& flipmObserverParams) {
  DEBUG_RESPONSE_ONCE(
    "module:RequestTranslator:FLIPMObserverParams:save_default",
    save(flipmObserverParams, File::getBHDir() + string("/Config/Robots/Default/flipmObserverParams.cfg")););
  DEBUG_RESPONSE_ONCE(
    "module:RequestTranslator:FLIPMObserverParams:save_robot",
    save(flipmObserverParams,
         File::getBHDir() + string("/Config/Robots/") + Global::getSettings().robot + string("/flipmObserverParams.cfg")););
  translator.updateFLIPMObserverParams(flipmObserverParams);
}

MAKE_MODULE(RequestTranslatorModule, dortmundWalkingEngine)

/**
 * @file Blackboard.cpp
 *
 * Implementation of a class representing the blackboard.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "Blackboard.h"
#include <cstring>
#include <cstdlib>

__attribute__((no_sanitize("undefined"))) Blackboard::Blackboard()
    : // Initialize all representations by themselves:

      // Personal Data
      thePersonalData(thePersonalData),

      // Infrastructure
      theJointData(theJointData), theJointRequest(theJointRequest), theSensorData(theSensorData), theKeyStates(theKeyStates),
      theLEDRequest(theLEDRequest), theImage(theImage), theCameraInfo(theCameraInfo), theFrameInfo(theFrameInfo),
      theCognitionFrameInfo(theCognitionFrameInfo), theRobotInfo(theRobotInfo), theOwnTeamInfo(theOwnTeamInfo),
      theOpponentTeamInfo(theOpponentTeamInfo), theRawGameInfo(theRawGameInfo), theGameInfo(theGameInfo),
      theTeamMateData(theTeamMateData), theMotionRobotHealth(theMotionRobotHealth), theRobotHealth(theRobotHealth),
      theTeamDataSenderOutput(theTeamDataSenderOutput), theUSRequest(theUSRequest), theThumbnail(theThumbnail),
      theFsrSensorData(theFsrSensorData),

      // Configuration
      theCameraSettings(theCameraSettings), theFieldDimensions(theFieldDimensions), theRobotDimensions(theRobotDimensions),
      theJointCalibration(theJointCalibration), theSensorCalibration(theSensorCalibration),
      theCameraCalibration(theCameraCalibration), theMassCalibration(theMassCalibration),
      theHardnessSettings(theHardnessSettings), theDamageConfiguration(theDamageConfiguration),
      theDamageConfigurationHead(theDamageConfigurationHead), theHeadLimits(theHeadLimits),
      theOdometryCorrectionTables(theOdometryCorrectionTables),

      // Perception
      theCameraMatrix(theCameraMatrix), theRobotCameraMatrix(theRobotCameraMatrix),
      theImageCoordinateSystem(theImageCoordinateSystem), theBallSpots(theBallSpots), theBallPercept(theBallPercept),
      theLineAnalysis(theLineAnalysis), theGoalPercept(theGoalPercept), theGroundContactState(theGroundContactState),
      theBodyContour(theBodyContour), theColorReference(theColorReference), theFieldBoundary(theFieldBoundary),
      theFeatures(theFeatures), theShapePercept(theShapePercept), theImageAcquisition(theImageAcquisition),
      theLinePercept(theLinePercept), thePenaltyMarkPercept(thePenaltyMarkPercept), thePlayerPercept(thePlayerPercept),
      theRandomForests(theRandomForests), theRefereePercept(theRefereePercept),

      // Modeling
      theArmContactModel(theArmContactModel), theFallDownState(theFallDownState), theBallModel(theBallModel),
      theBallModelAfterPreview(theBallModelAfterPreview), theCombinedWorldModel(theCombinedWorldModel),
      theGroundTruthWorldState(theGroundTruthWorldState), theGroundTruthBallModel(theGroundTruthBallModel),
      theRobotPose(theRobotPose), theRobotPoseAfterPreview(theRobotPoseAfterPreview),
      theRobotPoseSamples(theRobotPoseSamples), theFilteredRobotPose(theFilteredRobotPose),
      theFootContactModel(theFootContactModel), theGroundTruthRobotPose(theGroundTruthRobotPose),
      thePlayerModel(thePlayerModel), theGroundTruthPlayerModel(theGroundTruthPlayerModel),
      theFieldCoverage(theFieldCoverage), theSideConfidence(theSideConfidence), theOdometer(theOdometer),
      theOwnSideModel(theOwnSideModel), theWhistle(theWhistle), theAudioAcquisition(theAudioAcquisition),
      theAudioData(theAudioData), theLineLocalization(theLineLocalization), thePassHelper(thePassHelper),
      theCoordinatedPassRepresentation(theCoordinatedPassRepresentation),
      theFixedOdometryRobotPose(theFixedOdometryRobotPose),

      // BehaviorControl
      theActivationGraph(theActivationGraph), theBehaviorControlOutput(theBehaviorControlOutput),
      theBehaviorLEDRequest(theBehaviorLEDRequest), thePath(thePath), theGoalSymbols(theGoalSymbols), thePlan(thePlan),

      // Sensing
      theFilteredJointData(theFilteredJointData), theFilteredSensorData(theFilteredSensorData),
      theInertiaSensorData(theInertiaSensorData), theOrientationData(theOrientationData),
      theGroundTruthOrientationData(theGroundTruthOrientationData), theTorsoMatrix(theTorsoMatrix),
      theRobotModel(theRobotModel), theRobotBalance(theRobotBalance), theFsrData(theFsrData), theFsrZmp(theFsrZmp),
      theZMPModel(theZMPModel), theArmContact(theArmContact), theFootSupport(theFootSupport),

      // MotionControl
      theArmMotionEngineOutput(theArmMotionEngineOutput), theArmMotionRequest(theArmMotionRequest),
      theArmMovement(theArmMovement), theOdometryData(theOdometryData),
      theGroundTruthOdometryData(theGroundTruthOdometryData), theMotionRequest(theMotionRequest),
      theSpeedRequest(theSpeedRequest), theHeadAngleRequest(theHeadAngleRequest), theHeadMotionRequest(theHeadMotionRequest),
      theHeadJointRequest(theHeadJointRequest), theMotionSelection(theMotionSelection),
      theSpecialActionsOutput(theSpecialActionsOutput), theWalkingEngineOutput(theWalkingEngineOutput),
      theWalkingEngineStandOutput(theWalkingEngineStandOutput), theBikeEngineOutput(theBikeEngineOutput),
      theKickEngineOutput(theKickEngineOutput), theMotionInfo(theMotionInfo), theKinematicRequest(theKinematicRequest),
      theJoystickControl(theJoystickControl), theJoystickImageControl(theJoystickImageControl),
      theControllerParams(theControllerParams), theWalkingInfo(theWalkingInfo), theBodyTilt(theBodyTilt),
      theActualCoM(theActualCoM), theActualCoMRCS(theActualCoMRCS), theActualCoMFLIPM(theActualCoMFLIPM),
      theWalkingEngineParams(theWalkingEngineParams), theRefZMP(theRefZMP), theTargetCoM(theTargetCoM),
      theReferenceModificator(theReferenceModificator), theFootSteps(theFootSteps), theFootpositions(theFootpositions),
      theObservedFLIPMError(theObservedFLIPMError), theKinematicOutput(theKinematicOutput), theSpeedInfo(theSpeedInfo),
      thePatternGenRequest(thePatternGenRequest), theWalkCalibration(theWalkCalibration), theObservedError(theObservedError),
      theFLIPMObserverParams(theFLIPMObserverParams), theWalkGeneratorData(theWalkGeneratorData),
      theWalkKickGenerator(theWalkKickGenerator), thePlannedSteps(thePlannedSteps), theWalkLearner(theWalkLearner),
      theWalk2014Modifier(theWalk2014Modifier) {}

void Blackboard::operator=(const Blackboard& other) {
  memcpy((void*)this, (void*)&other, sizeof(Blackboard));
}

void* Blackboard::operator new(std::size_t size) {
  return calloc(1, size);
}

void Blackboard::operator delete(void* p) {
  return free(p);
}

void Blackboard::distract() {}

PROCESS_WIDE_STORAGE(Blackboard) Blackboard::theInstance = 0;

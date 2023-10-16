/**
 * @file KickEngine.cpp
 *
 * This file implements a module that creates kicking motions.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#include "KickEngine.h"
#include "KickEngineParameters.h"
#include "Core/System/File.h"
#include "Representations/MotionControl/KickRequest.h"

#include <dirent.h>
#include <cstdio>
#include <cstring>
#include <cerrno>

MAKE_MODULE(KickEngine, Motion Control)

KickEngine::KickEngine() {
  params.reserve(10);

  char dirname[260];

  sprintf(dirname, "%s/Config/Kicks/", File::getBHDir());
  DIR* dir = opendir(dirname);
  ASSERT(dir);
  struct dirent* file = readdir(dir);

  while (file != nullptr) {
    char name[260] = "";
    sprintf(name, "Kicks/%s", file->d_name);

    if (strstr(name, ".kmc")) {
      InMapFile stream(name);
      ASSERT(stream.exists());

      KickEngineParameters parameters;
      stream >> parameters;

      sprintf(name, "%s", file->d_name);
      for (int i = 0; i < 260; i++) {
        if (name[i] == '.') {
          name[i] = 0;
        }
      }
      strcpy(parameters.name, name);

      if (KickRequest::getKickMotionFromName(parameters.name) < KickRequest::newKick) {
        params.push_back(parameters);
      } else {
        OUTPUT_TEXT("Warning: KickRequest is missing the id for " << parameters.name);
        fprintf(stderr, "Warning: KickRequest is missing the id for %s \n", parameters.name);
      }
    }
    file = readdir(dir);
  }
  closedir(dir);

  for (int i = 0; i < KickRequest::newKick; ++i) {
    int id = -1;
    for (unsigned int p = 0; p < params.size(); ++p) {
      if (KickRequest::getKickMotionFromName(&params[p].name[0]) == i) {
        id = i;
        break;
      }
    }
    if (id == -1) {
      OUTPUT_TEXT("Warning: The kick motion file for id " << KickRequest::getName(static_cast<KickRequest::KickMotionID>(i))
                                                          << " is missing.");
      fprintf(stderr,
              "Warning: The kick motion file for id %s is missing. \n",
              KickRequest::getName(static_cast<KickRequest::KickMotionID>(i)));
    }
  }

  // This is needed for adding new kicks
  KickEngineParameters newKickMotion;
  strcpy(newKickMotion.name, "newKick");
  params.push_back(newKickMotion);
};

void KickEngine::update(KickEngineOutput& kickEngineOutput) {
  // Is the KickEngine activ?
  // Vector2<> test = supportfootCollisionOffset;
  MODIFY("parameters:KickEngineParams", theOmniKickParameters);

  data.addJointDataDebuggingPlot(theJointData, kickEngineOutput.jointRequest);
  if (theMotionSelection.ratios[MotionRequest::kick] > 0.f) {
    // Did the KickEngine start went active or is about to shutdown?
    if (theMotionSelection.ratios[MotionRequest::kick] < 1.f && !compensated) {
      compensate = true;
    }

    if (theMotionRequest.kickRequest.kickMotionType != KickRequest::none) {
      lastValidKickRequest = theMotionRequest.kickRequest;
    }
    data.robotModel = theRobotModel;
    // Do we need to wait befor we can do a kick?
    if (data.sitOutTransitionDisturbance(
          compensate, compensated, theInertiaSensorData, kickEngineOutput, theJointRequest, theFrameInfo)) {
      if (data.activateNewMotion(lastValidKickRequest, kickEngineOutput.isLeavingPossible) &&
          lastValidKickRequest.kickMotionType != KickRequest::none) {
        data.initData(theFrameInfo,
                      lastValidKickRequest,
                      params,
                      theJointData,
                      theTorsoMatrix,
                      kickEngineOutput.jointRequest,
                      theRobotDimensions,
                      theMassCalibration,
                      theDamageConfiguration,
                      theBallModel,
                      theOmniKickParameters);
        data.updateBallPosition(theBallModel);
        data.currentKickRequest = lastValidKickRequest;
        data.setExecutedKickRequest(kickEngineOutput.executedKickRequest);

        data.internalIsLeavingPossible = false;
        kickEngineOutput.isLeavingPossible = false;

        data.calcOdometryOffset(kickEngineOutput, theRobotModel); // to init it
        kickEngineOutput.odometryOffset = Pose2D();

        for (int i = JointData::LShoulderPitch; i < JointData::numOfJoints; ++i) {
          kickEngineOutput.jointRequest.jointHardness.hardness[i] = 100;
        }

        boostState = 0;
        kickEngineOutput.isStable = true;
      }

      // Is our Kick not over?
      if (data.checkPhaseTime(theFrameInfo, theJointData, theTorsoMatrix) &&
          ((data.relBallPosition - theBallModel.estimate.position).abs() <= theOmniKickParameters.exitThreshHold ||
           data.kickPhaseReached())) {
        data.calcPhaseState();
        data.calcPositions(theTorsoMatrix, lastValidKickRequest.kickMotionType);
        timeSinceLastPhase = theFrameInfo.time;
      }
      // Our current kick is over
      else {
        kickEngineOutput.isLeavingPossible = true;
        data.internalIsLeavingPossible = true;
      }

      // Is the current kick id valid, then calculate the jointRequest once for the balanceCom()
      if (data.calcJoints(kickEngineOutput.jointRequest, theRobotDimensions, theDamageConfiguration)) {
        data.balanceCOM(kickEngineOutput.jointRequest, theRobotDimensions, theMassCalibration);
        data.calcJoints(kickEngineOutput.jointRequest, theRobotDimensions, theDamageConfiguration);
        data.mirrorIfNecessary(kickEngineOutput.jointRequest);

        if (!theDamageConfiguration.dontBoost) {
          Pose3D real = theRobotModel.soleLeft.invert() * theRobotModel.soleRight;
          if (kickEngineOutput.executedKickRequest.mirror) {
            real.invert();
          }
          if (boostState == 0 && real.translation.x < -30) {
            boostState = 1;
          }
          if (boostState == 1 && real.translation.x > -20) {
            boostState = 2;
          }
          if (boostState >= 2 && boostState <= 4) {
            boostState += boostState < 4 ? 1 : 0;
            data.BOOST(kickEngineOutput.jointRequest, boostState);
            if (real.translation.x > 80) {
              boostState = 6;
            }
          }
        }

        data.calcOdometryOffset(kickEngineOutput, theRobotModel);
      }
      data.addGyroBalance(kickEngineOutput.jointRequest,
                          theJointCalibration,
                          theInertiaSensorData,
                          theMotionSelection.ratios[MotionRequest::kick]);
    }
  } else {
    compensated = false;
  }

  data.setEngineActivation(theMotionSelection.ratios[MotionRequest::kick]);
  data.ModifyData(theMotionRequest.kickRequest, kickEngineOutput.jointRequest, params);
}

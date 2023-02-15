/**
 * @file CoMProviderModule.cpp
 *
 * Module wrapper for the CoMProvider
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#include "CoMProviderModule.h"
#include "Core/Debugging/DebugDrawings.h"

CoMProviderModule::CoMProviderModule()
    : controller(
        // theJointAngles,
        // theWalkingEngineParams,
        // theJointRequest,
        theWalkingInfo,
        // theFootSteps,
        // theRobotModel,
        theActualCoMRCS) {}

void CoMProviderModule::update(ActualCoM& theActualCoM) {
  controller.updateActualCoM(theActualCoM);
  // PLOT("module:CoMProvider:JointData:LHipRoll", theJointAngles.angles[Joints::lHipRoll]);
  // PLOT("module:CoMProvider:JointData:LHipPitch", theJointAngles.angles[Joints::lHipPitch]);
  // PLOT("module:CoMProvider:JointData:LKneePitch", theJointAngles.angles[Joints::lKneePitch]);
  // PLOT("module:CoMProvider:JointData:LAnklePitch", theJointAngles.angles[Joints::lAnklePitch]);
  // PLOT("module:CoMProvider:JointData:LAnkleRoll", theJointAngles.angles[Joints::lAnkleRoll]);
  // PLOT("module:CoMProvider:JointData:LHipYawPitch", theJointAngles.angles[Joints::lHipYawPitch]);
  //
  // PLOT("module:CoMProvider:JointData:RHipRoll", theJointAngles.angles[Joints::rHipRoll]);
  // PLOT("module:CoMProvider:JointData:RHipPitch", theJointAngles.angles[Joints::rHipPitch]);
  // PLOT("module:CoMProvider:JointData:RKneePitch", theJointAngles.angles[Joints::rKneePitch]);
  // PLOT("module:CoMProvider:JointData:RAnklePitch", theJointAngles.angles[Joints::rAnklePitch]);
  // PLOT("module:CoMProvider:JointData:RAnkleRoll", theJointAngles.angles[Joints::rAnkleRoll]);
  // PLOT("module:CoMProvider:JointData:RHipYawPitch", theJointAngles.angles[Joints::rHipYawPitch]);
  // PLOT("module:CoMProvider:ActualCoM.x", theActualCoM.x);
  // PLOT("module:CoMProvider:ActualCoM.y", theActualCoM.y);
};

void CoMProviderModule::update(ActualCoMFLIPM& theActualCoMFLIPM) {
  controller.updateActualCoM(theActualCoMFLIPM);
};

MAKE_MODULE(CoMProviderModule, dortmundWalkingEngine)
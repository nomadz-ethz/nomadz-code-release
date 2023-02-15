/**
 * @file TiltController.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

/**
 * Copyright 2011, Oliver Urbann
 * All rights reserved.
 *
 * This file is part of MoToFlex.
 *
 * MoToFlex is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MoToFlex is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MoToFlex.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Contact e-mail: oliver.urbann@tu-dortmund.de
 */

#include "TiltController.h"
#include "Tools/DortmundWalkingEngine/Interpolator.h"
#include <algorithm>
using namespace std;

TiltController::TiltController(const InertiaSensorData& theInertiaSensorData,
                               const WalkingEngineParams& theWalkingEngineParams /*,
                                const JointCalibration		&theJointCalibration,
                                const JointRequest				&theJointRequest,
                                const SpeedInfo           &theSpeedInfo,
                                const TargetCoM				&theTargetCoM*/
                               )
    : theInertiaSensorData(theInertiaSensorData), theWalkingEngineParams(theWalkingEngineParams) /*,
                                                   theJointCalibration(theJointCalibration),
                                                   theJointRequest(theJointRequest),
                                                   theSpeedInfo(theSpeedInfo),
                                                   theTargetCoM(theTargetCoM)*/
{
  angleSum = 0;
  tiltPIDController.updateControlFactors(1.0, 0.0, 0.0);
  rollPIDController.updateControlFactors(1.0, 0.0, 0.0);
  tiltPIDController.setTimeStep(1.0);
  rollPIDController.setTimeStep(1.0);
}

TiltController::~TiltController(void) {}

DECLARE_INTERPOLATE_ARRAY(tiltControllerParams, float, 0.001f, 3);
DECLARE_INTERPOLATE_ARRAY(rollControllerParams, float, 0.001f, 3);

void TiltController::updateBodyTilt(BodyTilt& bodyTilt) {
  bodyTilt = lastBodyTilt;

#if 0
  	/*  Check joint constraints, if a joint is near to its min/max
		angles, adjust the body tilt */
	if (theJointRequest.jointAngles.angles[JointRequest::legLeft1]!=1000)
	{
		double diff1=0, diff2=0, diff=0, pBonus=0;

		if (theWalkingEngineParams.angleTiltP==0)
			pBonus=0.05;


		// Rotate if joint reach min/max
		// Left leg
		if (theJointRequest.jointAngles.angles[JointRequest::legLeft1]>theJointCalibration.joints[JointRequest::legLeft1].maxAngle)
			diff1=theJointRequest.jointAngles.angles[JointRequest::legLeft1]-theJointCalibration.joints[JointRequest::legLeft1].maxAngle;

		if (theJointRequest.jointAngles.angles[JointRequest::legLeft5]<theJointCalibration.joints[JointRequest::legLeft5].minAngle)
			diff2=theJointCalibration.joints[JointRequest::legLeft5].minAngle-theJointRequest.jointAngles.angles[JointRequest::legLeft5];

		angleSum.x-=theWalkingEngineParams.angleTiltP*max(diff1, diff2);

		// Right leg
		if (theJointRequest.jointAngles.angles[JointRequest::legRight1]>theJointCalibration.joints[JointRequest::legRight1].maxAngle)
			diff1=theJointRequest.jointAngles.angles[JointRequest::legRight1]-theJointCalibration.joints[JointRequest::legRight1].maxAngle;

		if (theJointRequest.jointAngles.angles[JointRequest::legRight5]<theJointCalibration.joints[JointRequest::legRight5].minAngle)
			diff2=theJointCalibration.joints[JointRequest::legRight5].minAngle-theJointRequest.jointAngles.angles[JointRequest::legRight5];

		angleSum.x+=theWalkingEngineParams.angleTiltP*max(diff1, diff2);

		// Rotate back if not
		if (theJointRequest.jointAngles.angles[JointRequest::legLeft5]>theJointCalibration.joints[JointRequest::legLeft5].minAngle &&
			theJointRequest.jointAngles.angles[JointRequest::legLeft1]<theJointCalibration.joints[JointRequest::legLeft1].maxAngle &&
			angleSum.x<0)
		{
			diff1=theJointCalibration.joints[JointRequest::legLeft1].maxAngle-theJointRequest.jointAngles.angles[JointRequest::legLeft1];
			diff2=theJointRequest.jointAngles.angles[JointRequest::legLeft5]-theJointCalibration.joints[JointRequest::legLeft5].minAngle;
			diff=min(diff1, diff2);
			if (std::abs(diff)>std::abs(angleSum.x))
				angleSum.x=0;
			else
				angleSum.x+=(theWalkingEngineParams.angleTiltP+pBonus)*diff;
		}

		if (theJointRequest.jointAngles.angles[JointRequest::legRight5]>theJointCalibration.joints[JointRequest::legRight5].minAngle &&
			theJointRequest.jointAngles.angles[JointRequest::legRight1]<theJointCalibration.joints[JointRequest::legRight1].maxAngle &&
			angleSum.x>0)
		{
			diff1=theJointCalibration.joints[JointRequest::legRight1].maxAngle-theJointRequest.jointAngles.angles[JointRequest::legRight1];
			diff2=theJointRequest.jointAngles.angles[JointRequest::legRight5]-theJointCalibration.joints[JointRequest::legRight5].minAngle;
			diff=min(diff1, diff2);
			if (std::abs(diff)>std::abs(angleSum.x))
				angleSum.x=0;
			else
				angleSum.x-=(theWalkingEngineParams.angleTiltP+pBonus)*diff;
		}
	}
#endif

  bodyTilt.x += (float)angleSum.x;

  lastBodyTilt = bodyTilt;

  if (std::abs(bodyTilt.x) < 1.0f && std::abs(bodyTilt.y) < 1.0f) {

    tiltPIDController.updateControlFactors(
      INTERPOLATE_ARRAY_ELEMENT(tiltControllerParams, theWalkingEngineParams.sensorControl.tiltControllerParams[0], 0),
      INTERPOLATE_ARRAY_ELEMENT(tiltControllerParams, theWalkingEngineParams.sensorControl.tiltControllerParams[1], 1),
      INTERPOLATE_ARRAY_ELEMENT(tiltControllerParams, theWalkingEngineParams.sensorControl.tiltControllerParams[2], 2));
    rollPIDController.updateControlFactors(
      INTERPOLATE_ARRAY_ELEMENT(rollControllerParams, theWalkingEngineParams.sensorControl.rollControllerParams[0], 0),
      INTERPOLATE_ARRAY_ELEMENT(rollControllerParams, theWalkingEngineParams.sensorControl.rollControllerParams[1], 1),
      INTERPOLATE_ARRAY_ELEMENT(rollControllerParams, theWalkingEngineParams.sensorControl.rollControllerParams[2], 2));

    if (std::abs(theInertiaSensorData.angle.x) < 1.0f && (theInertiaSensorData.angle.y < 1.0f)) {
      bodyTilt.x += (float)rollPIDController.getControllerOutput(theInertiaSensorData.angle.x, bodyTilt.x);
      bodyTilt.y += (float)tiltPIDController.getControllerOutput(theInertiaSensorData.angle.y, bodyTilt.y);
    } else {
      bodyTilt.x = 0.0f;
      bodyTilt.y = 0.0f;
    }
  } else {
    bodyTilt.x = 0.0f;
    bodyTilt.y = 0.0f;
  }
}

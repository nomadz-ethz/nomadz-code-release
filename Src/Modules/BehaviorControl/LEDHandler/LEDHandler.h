/**
 * @file LEDHandler.h
 *
 * This file implements a module that generates the LEDRequest from certain representations.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"

MODULE(LEDHandler)
REQUIRES(FrameInfo)
REQUIRES(GameInfo)
REQUIRES(RobotInfo)
REQUIRES(OwnTeamInfo)
REQUIRES(BallModel)
REQUIRES(GoalPercept)
REQUIRES(FilteredSensorData)
REQUIRES(TeamMateData)
REQUIRES(GroundContactState)
REQUIRES(BehaviorControlOutput)
REQUIRES(BehaviorLEDRequest)
REQUIRES(CombinedWorldModel)
PROVIDES_WITH_MODIFY_AND_OUTPUT(LEDRequest)
END_MODULE

class LEDHandler : public LEDHandlerBase {
private:
  void update(LEDRequest& ledRequest);

  void setEyeColor(LEDRequest& ledRequest, bool left, BehaviorLEDRequest::EyeColor col, LEDRequest::LEDState s);

  void setRightEar(LEDRequest& ledRequest);
  void setLeftEar(LEDRequest& ledRequest);
  void setLeftEye(LEDRequest& ledRequest);
  void setRightEye(LEDRequest& ledRequest);
  void setChestButton(LEDRequest& ledRequest);

public:
  LEDHandler() {}
};

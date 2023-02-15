/**
 * @file RobotInfo.h
 *
 * The file declares a class that encapsulates the structure RobotInfo defined in
 * the file RoboCupGameControlData.h that is provided with the GameController.
 * It also maps the robot's name on the robot's model.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * and <a href="mailto:aschreck@informatik.uni-bremen.de">André Schreck</a>
 */

#pragma once

#include "Core/Communication/RoboCupControlData.h"
#include "Core/Streams/AutoStreamable.h"
#include "Core/Enum.h"
#include "Core/Global.h"
#include "Core/Settings.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/robot_info.hpp"
#endif
STREAMABLE_DECLARE(RobotInfo)

STREAMABLE_ROS(RobotInfo, {
public:
  /** Update data from RoboCup GameCtrl data. */
  void setFromRoboCupData(RoboCup::RobotInfo * robotInfo) {
    penalty = robotInfo->penalty;
    secsTillUnpenalised = robotInfo->secsTillUnpenalised;
  }

  ENUM(NaoVersion, V33, V4, V5); // need to be sorted

  ENUM(NaoType, H21, H25); // need to be sorted

  ENUM(RobotFeature, hands, grippyFingers, wristYaws, zGyro, tactileHandSensores, tactileHeadSensores, headLEDs);

  bool hasFeature(const RobotFeature feature) const;

  NaoVersion naoVersion = V5;
  NaoType naoBodyType = H21;
  NaoType naoHeadType = H21;
  , FIELD_WRAPPER(int, 0, nomadz_msgs::msg::RobotInfo::number, number),       /**< Number of the robot. */
    FIELD_WRAPPER(uint8_t, 0, nomadz_msgs::msg::RobotInfo::penalty, penalty), /**< Penalty state of the robot. */
    FIELD_WRAPPER(uint8_t,
                  0,
                  nomadz_msgs::msg::RobotInfo::secs_till_unpenalised,
                  secsTillUnpenalised), /**< Seconds until penalty is over. */

    number = Global::settingsExist() ? Global::getSettings().playerNumber : 0;
});

/**
 * @file TeamRobot.cpp
 *
 * Implementation of a class representing a process that communicates with a remote robot via team communication.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "TeamRobot.h"
#include "Core/System/Time.h"
#define Drawings ::Drawings // Base class also defines Drawings, but need the global class

bool TeamRobot::main() {
  {
    SYNC;
    OUTPUT(idProcessBegin, bin, 't');

    DECLARE_DEBUG_DRAWING("representation:RobotPose", "drawingOnField");           // The robot pose
    DECLARE_DEBUG_DRAWING("representation:RobotPose:deviation", "drawingOnField"); // The robot pose
    DECLARE_DEBUG_DRAWING("origin:RobotPose", "drawingOnField");         // Set the origin to the robot's current position
    DECLARE_DEBUG_DRAWING("representation:BallModel", "drawingOnField"); // drawing of the ball model
    DECLARE_DEBUG_DRAWING("representation:CombinedWorldModel", "drawingOnField");     // drawing of the combined world model
    DECLARE_DEBUG_DRAWING("representation:PlayerModel:robots", "drawingOnField");     // drawing of the robots model a
    DECLARE_DEBUG_DRAWING("representation:PlayerModel:covariance", "drawingOnField"); // drawing of the robots model b
    DECLARE_DEBUG_DRAWING("representation:GoalPercept:Field", "drawingOnField");      // drawing of the goal percept
    DECLARE_DEBUG_DRAWING("representation:MotionRequest", "drawingOnField");          // drawing of a request walk vector
    DECLARE_DEBUG_DRAWING("representation:LineAnalysis:Field", "drawingOnField");
    DECLARE_PLOT("tgb");
    DECLARE_PLOT("tob");

    int teamColor = 0, swapSides = 0;
    MODIFY("teamColor", teamColor);
    MODIFY("swapSides", swapSides);

    if (Time::getTimeSince(robotPoseReceived) < 1000) {
      robotPose.draw(!teamColor);
    }
    if (Time::getTimeSince(playerModelReceived) < 1000) {
      playerModel.draw();
    }
    if (Time::getTimeSince(ballModelReceived) < 1000) {
      ballModel.draw();
    }
    if (Time::getTimeSince(combinedWorldModelReceived) < 1000) {
      combinedWorldModel.draw();
    }
    if (Time::getTimeSince(goalPerceptReceived) < 1000) {
      goalPercept.draw();
    }
    if (Time::getTimeSince(motionRequestReceived) < 1000) {
      motionRequest.draw();
    }
    if (Time::getTimeSince(lineAnalysisReceived) < 1000) {
      lineAnalysis.drawOnField(fieldDimensions, 0);
    }

    if (swapSides ^ teamColor) {
      ORIGIN("field polygons", 0, 0, pi2); // hack: swap sides!
    }
    fieldDimensions.draw();
    fieldDimensions.drawPolygons(teamColor);
    int fontSize = 12;
    int spacing = 320;
    int xRight = 2200;

    DECLARE_DEBUG_DRAWING("robotState", "drawingOnField"); // text decribing the state of the robot
    int lineY = 3400;
    DRAWTEXT(
      "robotState", -5100, lineY, fontSize, ColorClasses::white, "batteryLevel: " << robotHealth.batteryLevel << " %");
    DRAWTEXT(
      "robotState", xRight, lineY, fontSize, ColorClasses::white, "role: " << BehaviorStatus::getName(behaviorStatus.role));
    lineY -= spacing;
    DRAWTEXT("robotState",
             -5100,
             lineY,
             fontSize,
             ColorClasses::white,
             "temperatures: joint: " << robotHealth.maxJointTemperature << " C, cpu: " << robotHealth.cpuTemperature
                                     << " C, board: " << robotHealth.boardTemperature << " C");
    lineY -= spacing;
    DRAWTEXT("robotState",
             -5100,
             lineY,
             fontSize,
             ColorClasses::white,
             "rates: cognition: " << roundNumberToInt(robotHealth.cognitionFrameRate)
                                  << " fps, motion: " << roundNumberToInt(robotHealth.motionFrameRate) << " fps");
    if (ballModel.timeWhenLastSeen) {
      DRAWTEXT("robotState",
               xRight,
               lineY,
               fontSize,
               ColorClasses::white,
               "ballLastSeen: " << Time::getRealTimeSince(ballModel.timeWhenLastSeen) << " ms");
    } else {
      DRAWTEXT("robotState", xRight, lineY, fontSize, ColorClasses::white, "ballLastSeen: never");
    }
    // DRAWTEXT("robotState", -5100, lineY, font_size, ColorClasses::white, "ballPercept: " <<
    // ballModel.lastPerception.position.x << ", " << ballModel.lastPerception.position.y);
    lineY -= spacing;
    DRAWTEXT("robotState", -5100, lineY, fontSize, ColorClasses::white, "memory usage: " << robotHealth.memoryUsage << " %");
    if (goalPercept.timeWhenGoalPostLastSeen) {
      DRAWTEXT("robotState",
               xRight,
               lineY,
               fontSize,
               ColorClasses::white,
               "goalLastSeen: " << Time::getRealTimeSince(goalPercept.timeWhenGoalPostLastSeen) << " ms");
    } else {
      DRAWTEXT("robotState", xRight, lineY, fontSize, ColorClasses::white, "goalLastSeen: never");
    }

    lineY -= spacing;

    DRAWTEXT("robotState",
             -5100,
             lineY,
             fontSize,
             ColorClasses::white,
             "load average: " << (float(robotHealth.load[0]) / 10.f) << " " << (float(robotHealth.load[1]) / 10.f) << " "
                              << (float(robotHealth.load[2]) / 10.f));
    DRAWTEXT("robotState", -4100, 0, fontSize, ColorRGBA(255, 255, 255, 50), robotHealth.robotName);

    DECLARE_DEBUG_DRAWING("robotOffline", "drawingOnField"); // A huge X to display the online/offline state of the robot
    if (Time::getTimeSince(robotPoseReceived) > 500 || (Time::getTimeSince(isPenalizedReceived) < 1000 && isPenalized) ||
        (Time::getTimeSince(hasGroundContactReceived) < 1000 && !hasGroundContact) ||
        (Time::getTimeSince(isUprightReceived) < 1000 && !isUpright)) {
      LINE("robotOffline", -5100, 3600, 5100, -3600, 50, Drawings::ps_solid, ColorRGBA(0xff, 0, 0));
      LINE("robotOffline", 5100, 3600, -5100, -3600, 50, Drawings::ps_solid, ColorRGBA(0xff, 0, 0));
    }
    if (Time::getTimeSince(isPenalizedReceived) < 1000 && isPenalized) {
      // Draw a text in red letters to tell that the robot is penalized
      DRAWTEXT("robotState", -2000, 0, fontSize * 1.5, ColorClasses::red, "PENALIZED");
    }
    if (Time::getTimeSince(hasGroundContactReceived) < 1000 && !hasGroundContact) {
      // Draw a text in red letters to tell that the robot doesn't have ground contact
      DRAWTEXT("robotState", 300, 0, fontSize * 1.5, ColorClasses::red, "NO GROUND CONTACT");
    }
    if (Time::getTimeSince(isUprightReceived) < 1000 && !isUpright) {
      // Draw a text in red letters to tell that the robot is fallen down
      DRAWTEXT("robotState", 300, 0, fontSize * 1.5, ColorClasses::red, "NOT UPRIGHT");
    }

    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager", OUTPUT(idDrawingManager, bin, Global::getDrawingManager()););
    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager3D",
                        OUTPUT(idDrawingManager3D, bin, Global::getDrawingManager3D()););
    DEBUG_RESPONSE_ONCE("automated requests:StreamSpecification",
                        OUTPUT(idStreamSpecification, bin, Global::getStreamHandler()););

    OUTPUT(idProcessFinished, bin, 't');
    teamOut.moveAllMessages(teamIn);
  }
  SystemCall::sleep(50);
  return false;
}

TeamRobot::TeamRobot(const char* name, int number) : RobotConsole(teamIn, teamOut), number(number) {
  strcpy(this->name, name);
  mode = SystemCall::teamRobot;
  fieldDimensions.load();
}

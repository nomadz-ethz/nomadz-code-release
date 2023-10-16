/**
 * @file LocalRobot.h
 *
 * Declaration of LocalRobot.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 * and <A href="mailto:kspiess@tzi.de">Kai Spiess</A>
 */

#pragma once

#include "Controller/RobotConsole.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Modeling/GroundTruthWorldState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/OrientationData.h"
#include "Oracle.h"

/**
 * @class LocalRobot
 *
 * A process that is instantiated to either directly control a physical robot,
 * a simulated one, or to replay a logfile.
 */
class LocalRobot : public RobotConsole {
private:
  DEBUGGING;
  Image image;                          /**< The simulated image sent to the robot code. */
  CameraInfo cameraInfo;                /**< The information about the camera that took the image sent to the robot code. */
  JointData jointData;                  /**< The simulated joint measurements sent to the robot code. */
  SensorData sensorData;                /**< The simulated sensor data sent to the robot code. */
  GroundTruthRobotPose robotPose;       /**< The simulated ground truth robot pose sent to the robot code. */
  GroundTruthWorldState worldState;     /**< The simulated ground truth world state sent to the robot code. */
  GroundTruthOdometryData odometryData; /**< The simulated odometry data sent to the robot code. */
  GroundTruthOrientationData orientationData; /**< The simulated orientation data sent to the robot code. */
  unsigned nextImageTimeStamp,                /**< The theoretical timestamp of the next image to be calculated. */
    imageLastTimeStampSent,                   /**< The timestamp of the last sent image. */
    jointLastTimeStampSent;                   /**< The timestamp of the last sent joint data. */
  Oracle oracle;                              /**< The interface to simulated objects. */
  Semaphore updateSignal;                     /**< A signal used for synchronizing main() and update(). */
  Semaphore updatedSignal;                    /**< A signal used for yielding processing time to main(). */
  SimRobotCore2::Body* puppet; /**< A pointer to the puppet when there is one during logfile replay. Otherwise 0. */
public:
  /**
   * Constructor.
   */
  LocalRobot();

  /**
   * The function is called from the framework once in every frame
   */
  virtual bool main();

  /**
   * The function must be called to exchange data with SimRobot.
   * It sends the motor commands to SimRobot and acquires new sensor data.
   */
  void update();
};

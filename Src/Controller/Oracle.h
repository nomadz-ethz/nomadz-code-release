/**
 * @file Oracle.h
 *
 * Declaration of class Oracle for SimRobotQt.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#pragma once

#include "SimRobotCore2.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/USRequest.h"
#include "Core/Math/Vector2.h"
#include "Core/Math/Pose3D.h"

class RoboCupCtrl;
class Pose2D;
class OdometryData;
class Image;
class SensorData;
class JointData;
class RobotPose;
class GroundTruthWorldState;
class BallModel;
class OrientationData;
class ImageInfo;
class ImageRequest;
class CameraInfo;

/**
 * An interface to a simulated robot (and its ball).
 */
class Oracle {
public:
  /** enum which declares the different types of balls leaving the field */
  enum BallOut {
    NONE,
    GOAL_BY_AWAY,
    GOAL_BY_HOME,
    OUT_BY_AWAY,
    OUT_BY_HOME,
    CORNER_KICK_AWAY,
    CORNER_KICK_HOME,
    GOAL_FREE_KICK_AWAY,
    GOAL_FREE_KICK_HOME
  };

  /** Default constructor */
  Oracle();

  /** Destructor */
  ~Oracle();

  /**
   * Initilizes common settings of the oracle.
   */
  static void init();

  /**
   * Creates an oracle for the given robot.
   * @param robot The robot to create an oracle for
   */
  void init(SimRobot::Object* robot);

  /**
   * Sets the only ball used to create the ball model.
   */
  static void setBall(SimRobot::Object* ball);

  /**
   * Proclaims which robot touched the ball at last
   * @param robot The robot
   */
  static void setLastBallContactRobot(SimRobot::Object* robot);

  /**
   * Get position of last ball contact
   * @return last ball contact (rotation == 0 -> away)
   */
  static Pose2D getLastBallContactRobot();

  /**
   * Determines the pose of the simulated robot.
   * @param robotPose The determined pose of the robot.
   */
  void getRobotPose(RobotPose& robotPose) const;

  /**
   * Determines all robot states as well as the ball state.
   * @param worldState The determined world state.
   */
  void getWorldState(GroundTruthWorldState& groundTruthWorldState) const;

  /**
   * Determines the odometry data of the simulated robot.
   * @param robotPose The pose of the robot.
   * @param odometryData The determined odometry data of the robot.
   */
  void getOdometryData(const RobotPose& robotPose, OdometryData& odometryData) const;

  /**
   * Determines the ball position in the scene (not considering the robot's color).
   * @param ballPosition The position of the ball
   */
  static void getAbsoluteBallPosition(Vector2<>& ballPosition);

  /**
   * Determines the camera image of the simulated robot.
   * @param image The determined image.
   * @param cameraInfo The information about the camera that took the image.
   */
  void getImage(Image& image, CameraInfo& cameraInfo);

  /**
   * Determines the current joint data of the simulated robot and sets new ones.
   * @param jointRequest The joint data to set.
   * @param jointData The determined joint data.
   */
  void getAndSetJointData(const JointData& jointRequest, JointData& jointData) const;

  /**
   * Sets the values off all joint actuators.
   * @param jointRequest The joint data to set.
   */
  void setJointData(const JointData& jointRequest) const;

  /**
   * Toggles between the two cameras.
   */
  void toggleCamera();

  /**
   * Determines the sensor data of the simulated robot.
   * @param sensorData The determined sensor data.
   * @param usRequest The request that determines which sonars are read.
   */
  void getSensorData(SensorData& sensorData, const USRequest& usRequest);

  /**
   * Determines "ground truth" orientation data.
   * @param sensorData The current set of sensor data.
   * @param orientationData The determined orientation data.
   */
  void getOrientationData(const SensorData& sensorData, OrientationData& orientationData);

  /**
   * Moves and rotates the robot to an absolute pose
   * @param pos The position to move the robot to
   * @param rot The target rotation (as angle axis; in radian)
   * @param changeRotation Whether the rotation of the robot should be changed or not
   */
  void moveRobot(const Vector3<>& pos, const Vector3<>& rot, bool changeRotation);

  /**
   * Enables or disables the physics simulation of the body
   * @param enable Whether to enable or disable the physics simulation
   */
  void enablePhysics(bool enable);

  /**
   * Moves the ball to the given position
   * @param pos The position to move the ball to
   * @param resetDynamics Reset dynamics of object after moving.
   */
  static void moveBall(const Vector3<>& pos, bool resetDynamics = false);

  /**
   * Update the ball position based on the rules.
   */
  static BallOut updateBall();

private:
  SimRobot::Application* application;

  bool homeTeam; /**< Whether this robot is the home team (left half as seen from GameController, +x in SimRobot) or not. */
  int robotNumber;                               /**< The number of this robot. */
  SimRobot::Object* robot;                       /**< The simulated roboter object. */
  SimRobot::Object* leftFoot;                    /**< The simulated left foot of the roboter. */
  SimRobot::Object* rightFoot;                   /**< The simulated right foot of the roboter. */
  static SimRobot::Object* ball;                 /**< The simulated ball. */
  std::vector<SimRobot::Object*> homeTeamRobots; /**< The simulated robots on the home team (excluding this robot). */
  std::vector<SimRobot::Object*> awayTeamRobots; /**< The simulated robots on the away team (excluding this robot). */

  Vector2<> lastBallPosition; /**< The last ball position on field to determine the velocity of the ball. */
  Pose3D lastRobotPose3D,     /**< The last robot pose in 3-D. */
    robotPose3D;              /**< A buffer for the current 3-D robot pose. */
  unsigned int lastTimeStamp; /** The time stamp of the previous iteration. */

  SimRobot::Object* jointSensors[JointData::numOfJoints];   /**< The handles to the sensor ports of the joints. */
  SimRobot::Object* jointActuators[JointData::numOfJoints]; /**< The handles to the actuator ports of the joints. */
  SimRobot::Object* cameraSensor;                           /**< The handle to the sensor port of the selected camera. */
  SimRobot::Object* upperCameraSensor;                      /**< The handle to the sensor port of the upper camera. */
  SimRobot::Object* lowerCameraSensor;                      /**< The handle to the sensor port of the lower camera. */
  SimRobot::Object* accSensor;           /**< The handle to the sensor port of the virtual accelerometer. */
  SimRobot::Object* gyroSensor;          /**< The handle to the sensor port of the virtual gyrosope. */
  SimRobot::Object* leftUsSensor;        /** The handle to the sensor port of the virtual us sensor */
  SimRobot::Object* rightUsSensor;       /** The handle to the sensor port of the virtual us sensor */
  SimRobot::Object* centerLeftUsSensor;  /** The handle to the sensor port of the virtual us sensor */
  SimRobot::Object* centerRightUsSensor; /** The handle to the sensor port of the virtual us sensor */

  static SimRobotCore2::SensorPort* activeCameras[12]; /**< An array of all activated cameras */
  static unsigned activeCameraCount;                   /**< Total count of constructed oracles */
  unsigned activeCameraIndex;                          /**< Index of this robot in the \c activeCameras array */

  static Pose2D lastBallContactPose; /**< Position were the last ball contact of a robot took place, orientation is toward
                                        opponent goal (0/180 degress). */

  JointCalibration jointCalibration;      /**< The simulated robot is perfectly calibrated, but we need the signs. */
  static FieldDimensions fieldDimensions; /**< The field dimensions for automatically placing the ball. */
  static CameraInfo upperCameraInfo;      /**< Information about the upper camera. */
  static CameraInfo lowerCameraInfo;      /**< Information about the lower camera. */

  /**
   * Adds jitter to a sensor value.
   */
  float addJitter(float value);

  /**
   * Determines the two-dimensional position of a SimRobot object without team color rotation.
   * @param obj The object of which the position will be determined.
   */
  static Vector2<> getPosition(SimRobot::Object* obj);

  /**
   * Determines the three-dimensional position of a SimRobot object without team color rotation.
   * @param obj The object of which the position will be determined.
   */
  static Vector3<> getPosition3D(SimRobot::Object* obj);

  /**
   * Mirrors a pose about the origin.
   * @param pose The pose to be mirrored.
   */
  static void mirrorPose(Pose2D& pose);

  /**
   * Determines the two-dimensional pose of a SimRobot object without team color rotation.
   * @param obj The object of which the pose will be determined.
   * @param pose2D The two-dimensional pose of the specified object.
   * @return Is the robot upright?
   */
  bool getPose2D(SimRobot::Object* obj, Pose2D& pose2D) const;

  /**
   * Determines the three-dimensional pose of a SimRobot object without team color rotation.
   * @param obj The object of which the pose will be determined.
   * @param pose3D The three-dimensional pose of the specified object.
   */
  void getPose3D(SimRobot::Object* obj, Pose3D& pose3D) const;

  /**
   * Determines whether a robot is member of the home or away team
   * @param obj The robot
   * @return \c true if the robot is in the home team; \c false otherwise
   */
  static bool isHomeTeam(SimRobot::Object* obj);

  /**
   * Determines a robot's number
   * @param obj The robot
   * @return The number
   */
  static int getNumber(const SimRobot::Object* obj);
};

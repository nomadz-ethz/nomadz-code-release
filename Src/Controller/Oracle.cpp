/**
 * @file Oracle.cpp
 *
 * Implementation of class Oracle for SimRobotQt.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#include <QString>
#include <QVector>

#include <algorithm>

#include "Oracle.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/TeamComm3DCtrl.h"
#include "Core/System/SystemCall.h"
#include "Core/System/Time.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/GroundTruthWorldState.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/OrientationData.h"
#include "Core/Streams/InStreams.h"

SimRobotCore2::SensorPort* Oracle::activeCameras[12]; // should be 10, but TeamComm3DCtrl allocates two unsued ones
unsigned Oracle::activeCameraCount = 0;

SimRobot::Object* Oracle::ball = 0;
Pose2D Oracle::lastBallContactPose;
FieldDimensions Oracle::fieldDimensions;
CameraInfo Oracle::upperCameraInfo;
CameraInfo Oracle::lowerCameraInfo;

Oracle::Oracle()
    : homeTeam(false), robot(0), leftFoot(0), rightFoot(0), lastTimeStamp(0), activeCameraIndex(activeCameraCount++) {
  ASSERT(activeCameraCount <= sizeof(activeCameras) / sizeof(activeCameras[0]));
}

Oracle::~Oracle() {
  --activeCameraCount;
}

void Oracle::init(SimRobot::Object* robot) {
  ASSERT(this->robot == 0);
  this->robot = (SimRobotCore2::Object*)robot;
  application = RoboCupCtrl::application ? RoboCupCtrl::application : TeamComm3DCtrl::application;

  // get the robot's team color
  homeTeam = isHomeTeam(robot);
  robotNumber = getNumber(robot);

  // get feet (for pose and odometry)
  QVector<QString> parts;
  parts.resize(1);
  parts[0] = "RFoot";
  VERIFY(rightFoot = (SimRobotCore2::Body*)application->resolveObject(parts, robot, SimRobotCore2::body));
  parts[0] = "LFoot";
  VERIFY(leftFoot = (SimRobotCore2::Body*)application->resolveObject(parts, robot, SimRobotCore2::body));

  // get joints
  parts.resize(1);
  QString position(".position");
  for (int i = 0; i < JointData::numOfJoints; ++i) {
    parts[0] = QString(JointData::getName(JointData::Joint(i))) + position;
    jointSensors[i] = (SimRobotCore2::SensorPort*)application->resolveObject(parts, robot, SimRobotCore2::sensorPort);
    jointActuators[i] = (SimRobotCore2::ActuatorPort*)application->resolveObject(parts, robot, SimRobotCore2::actuatorPort);
  }

  // imu sensors
  parts.resize(1);
  parts[0] = "Gyroscope.angularVelocities";
  gyroSensor = application->resolveObject(parts, robot, SimRobotCore2::sensorPort);

  parts[0] = "Accelerometer.acceleration";
  accSensor = application->resolveObject(parts, robot, SimRobotCore2::sensorPort);

  // cameras
  parts[0] = "CameraTop.image";
  upperCameraSensor = application->resolveObject(parts, robot, SimRobotCore2::sensorPort);
  parts[0] = "CameraBottom.image";
  lowerCameraSensor = application->resolveObject(parts, robot, SimRobotCore2::sensorPort);
  cameraSensor = lowerCameraSensor;
  activeCameras[activeCameraIndex] = (SimRobotCore2::SensorPort*)cameraSensor;

  // sonars
  parts[0] = "SonarLeft.distance";
  leftUsSensor = (SimRobotCore2::SensorPort*)application->resolveObject(parts, robot, SimRobotCore2::sensorPort);
  parts[0] = "SonarRight.distance";
  rightUsSensor = (SimRobotCore2::SensorPort*)application->resolveObject(parts, robot, SimRobotCore2::sensorPort);
  parts[0] = "SonarCenterLeft.distance";
  centerLeftUsSensor = (SimRobotCore2::SensorPort*)application->resolveObject(parts, robot, SimRobotCore2::sensorPort);
  parts[0] = "SonarCenterRight.distance";
  centerRightUsSensor = (SimRobotCore2::SensorPort*)application->resolveObject(parts, robot, SimRobotCore2::sensorPort);

  // load calibration
  InMapFile stream("jointCalibration.cfg");
  ASSERT(stream.exists());
  stream >> jointCalibration;

  // get pointers to other robots in "robots" group
  SimRobot::Object* group = application->resolveObject("RoboCup.robots", SimRobotCore2::compound);
  for (unsigned currentRobot = 0, count = application->getObjectChildCount(*group); currentRobot < count; ++currentRobot) {
    SimRobot::Object* robot = application->getObjectChild(*group, currentRobot);
    const int number = getNumber(robot);
    if (number != robotNumber) {
      if (number <= 5) {
        homeTeamRobots.push_back(robot);
      } else {
        awayTeamRobots.push_back(robot);
      }
    }
  }
  // get pointers to other robots in "extras" group
  group = application->resolveObject("RoboCup.extras", SimRobotCore2::compound);
  if (group != nullptr) {
    for (unsigned currentRobot = 0, count = application->getObjectChildCount(*group); currentRobot < count; ++currentRobot) {
      SimRobot::Object* robot = application->getObjectChild(*group, currentRobot);
      const int number = getNumber(robot);
      if (number != robotNumber) {
        if (number <= 5) {
          homeTeamRobots.push_back(robot);
        } else {
          awayTeamRobots.push_back(robot);
        }
      }
    }
  }
}

Oracle::BallOut Oracle::updateBall() {
  if (!ball) {
    return NONE;
  }

  BallOut result = NONE;
  Vector2<> ballPos = getPosition(ball);
  if (!fieldDimensions.isInsideField(ballPos)) {
    if (fabs(ballPos.y) < fieldDimensions.yPosLeftGoal) {
      result = ballPos.x > fieldDimensions.xPosOpponentGroundline ? GOAL_BY_AWAY : GOAL_BY_HOME;
    } else if (fabs(ballPos.y) < fieldDimensions.yPosLeftSideline - 20.f) {
      if (ballPos.x > 0) {
        result = lastBallContactPose.rotation == 0 ? GOAL_FREE_KICK_HOME : CORNER_KICK_AWAY;
      } else {
        result = lastBallContactPose.rotation == 0 ? CORNER_KICK_HOME : GOAL_FREE_KICK_AWAY;
      }
    } else {
      result = lastBallContactPose.rotation == 0 ? OUT_BY_AWAY : OUT_BY_HOME;
    }
  }

  return result;
}

void Oracle::init() {
  fieldDimensions.load();
  InMapFile upperStream("upperCameraInfo.cfg");
  ASSERT(upperStream.exists());
  upperStream >> upperCameraInfo;
  InMapFile lowerStream("lowerCameraInfo.cfg");
  ASSERT(lowerStream.exists());
  lowerStream >> lowerCameraInfo;
}

void Oracle::setBall(SimRobot::Object* ball) {
  Oracle::ball = ball;
}

void Oracle::setLastBallContactRobot(SimRobot::Object* robot) {
  lastBallContactPose = Pose2D(isHomeTeam(robot) ? pi : 0, getPosition(robot));
}
Pose2D Oracle::getLastBallContactRobot() {
  return lastBallContactPose;
}

void Oracle::getRobotPose(RobotPose& robotPose) const {
  ASSERT(rightFoot && leftFoot);

  getPose2D(robot, (Pose2D&)robotPose);
  robotPose.translation = (getPosition(leftFoot) + getPosition(rightFoot)) * 0.5f;

  if (homeTeam) {
    mirrorPose(robotPose);
  }

  robotPose.validity = 1.f;
  robotPose.deviation = 1.f;
}

void Oracle::getWorldState(GroundTruthWorldState& groundTruthWorldState) const {
  // Clear everything
  groundTruthWorldState.teammates.clear();
  groundTruthWorldState.opponents.clear();
  groundTruthWorldState.balls.clear();

  // Time
  groundTruthWorldState.time = Time::getCurrentSystemTime();

  // Ball
  if (ball) {
    groundTruthWorldState.balls.push_back(homeTeam ? -getPosition3D(ball) : getPosition3D(ball));
  }

  // Own pose
  getPose2D(robot, (Pose2D&)groundTruthWorldState.ownPose);
  groundTruthWorldState.ownPose.translation = (getPosition(leftFoot) + getPosition(rightFoot)) * 0.5f;
  if (homeTeam) {
    mirrorPose(groundTruthWorldState.ownPose);
  }

  // Home team players
  for (SimRobot::Object* robot : homeTeamRobots) {
    GroundTruthWorldState::Player player;
    player.number = getNumber(robot);
    player.upright = getPose2D(robot, player.pose);
    if (homeTeam) {
      mirrorPose(player.pose);
    }
    if (homeTeam) {
      groundTruthWorldState.teammates.push_back(player);
    } else {
      groundTruthWorldState.opponents.push_back(player);
    }
  }

  // Away team players
  for (SimRobot::Object* robot : awayTeamRobots) {
    GroundTruthWorldState::Player player;
    player.number = getNumber(robot) - 5;
    player.upright = getPose2D(robot, player.pose);
    if (homeTeam) {
      mirrorPose(player.pose);
    }
    if (homeTeam) {
      groundTruthWorldState.opponents.push_back(player);
    } else {
      groundTruthWorldState.teammates.push_back(player);
    }
  }
}

void Oracle::getOdometryData(const RobotPose& robotPose, OdometryData& odometryData) const {
  ASSERT(robot);
  (Pose2D&)odometryData = robotPose;
  if (homeTeam) {
    mirrorPose(odometryData);
  }
}

void Oracle::getAbsoluteBallPosition(Vector2<>& ballPosition) {
  if (ball) {
    ballPosition = getPosition(ball);
  }
}

void Oracle::getImage(Image& image, CameraInfo& cameraInfo) {
  ASSERT(robot);

  if (cameraSensor) {
    ((SimRobotCore2::SensorPort*)cameraSensor)->renderCameraImages(activeCameras, activeCameraCount);

    ASSERT(!image.isReference);
    if (cameraSensor == upperCameraSensor) {
      cameraInfo = upperCameraInfo;
    } else {
      cameraInfo = lowerCameraInfo;
    }

    image.setResolution(cameraInfo.width, cameraInfo.height);
    const int w = image.width;
    const int h = image.height;

    const int w3 = w * 3, w2 = image[1] - image[0];
    unsigned char* src = (unsigned char*)((SimRobotCore2::SensorPort*)cameraSensor)->getValue().byteArray;
    unsigned char* srcLineEnd = src;
    Image::Pixel* destBegin = image[0];
    Image::Pixel* dest;
    int r1, g1, b1, yy, cr;
    for (int y = h - 1; y >= 0; --y) {
      for (srcLineEnd += w3, dest = destBegin + y * w2; src < srcLineEnd;) {
        for (int i = 0; i < 4; ++i) {
          yy = 306 * (r1 = *(src++));
          cr = 130560 - 429 * (g1 = *(src++)) + 512 * r1;
          dest->cb = (unsigned char)((130560 - 173 * r1 - 339 * g1 + 512 * (b1 = *(src++))) >> 10);
          yy += 117 * b1 + 601 * g1;
          cr -= 83 * b1;
          dest->y = dest->yCbCrPadding = (unsigned char)(yy >> 10);
          (dest++)->cr = (unsigned char)(cr >> 10);
        }
      }
    }
  }

  image.timeStamp = Time::getCurrentSystemTime();
}

void Oracle::getAndSetJointData(const JointData& jointRequest, JointData& jointData) const {
  ASSERT(robot);

  for (int i = 0; i < JointData::numOfJoints; ++i) {
    // Get angles
    if (jointSensors[i]) {
      jointData.angles[i] =
        float(((SimRobotCore2::SensorPort*)jointSensors[i])->getValue().floatValue /* * jointCalibration.joints[i].sign */ -
              jointCalibration.joints[i].offset);
    }

    // Set angles
    const float& targetAngle(jointRequest.angles[i]);
    if (targetAngle != JointData::off && targetAngle != JointData::ignore && jointActuators[i]) { // if joint does exist
      ((SimRobotCore2::ActuatorPort*)jointActuators[i])->setValue((targetAngle) + jointCalibration.joints[i].offset);
    }
  }
  jointData.timeStamp = Time::getCurrentSystemTime();
}

void Oracle::setJointData(const JointData& jointRequest) const {
  ASSERT(robot);
  for (int i = 0; i < JointData::numOfJoints; ++i) {
    // Set angles
    const float& targetAngle(jointRequest.angles[i]);
    if (targetAngle != JointData::off && targetAngle != JointData::ignore && jointActuators[i]) { // if joint does exist
      ((SimRobotCore2::ActuatorPort*)jointActuators[i])->setValue((targetAngle) + jointCalibration.joints[i].offset);
    }
  }
}

void Oracle::toggleCamera() {
  cameraSensor = cameraSensor == lowerCameraSensor ? upperCameraSensor : lowerCameraSensor;
  activeCameras[activeCameraIndex] = (SimRobotCore2::SensorPort*)cameraSensor;
}

void Oracle::getSensorData(SensorData& sensorData, const USRequest& usRequest) {
  ASSERT(robot);

  sensorData.timeStamp = Time::getCurrentSystemTime();

  // Gyro
  const float* floatArray = ((SimRobotCore2::SensorPort*)gyroSensor)->getValue().floatArray;
  sensorData.data[SensorData::gyroX] = floatArray[0] + 0 * 12.f; // 12.f = bias
  sensorData.data[SensorData::gyroY] = floatArray[1] + 0 * 6.f;  // 6.f = bias
  sensorData.data[SensorData::gyroZ] = floatArray[2];            // nao style :(

  // Acc
  floatArray = ((SimRobotCore2::SensorPort*)accSensor)->getValue().floatArray;
  sensorData.data[SensorData::accX] = -1 * floatArray[0] + 0 * 0.1f;  // 0.1f = bias
  sensorData.data[SensorData::accY] = -1 * floatArray[1] + 0 * 0.2f;  // 0.2f = bias
  sensorData.data[SensorData::accZ] = -1 * floatArray[2] + 0 * 0.05f; // 0.05f = bias

  // angle
  {
    float position[3];
    float world2robot[3][3];
    ((SimRobotCore2::Body*)robot)->getPose(position, world2robot);
    sensorData.data[SensorData::angleX] = float(atan2(world2robot[1][2], world2robot[2][2]));
    sensorData.data[SensorData::angleY] = -float(atan2(world2robot[0][2], world2robot[2][2]));
  }

  // Battery
  sensorData.data[SensorData::batteryLevel] = 1.0f;

  // ultrasonic (model approximation. not absolutely correct in reality)

  static const float scale = 1000; // meter to millimeter
  if (usRequest.receiveMode != -1) {
    sensorData.usActuatorMode = (SensorData::UsActuatorMode) static_cast<int>(usRequest.receiveMode);

    // FIXME simulate additional sensor values
    for (int i = SensorData::usL; i < SensorData::usREnd; ++i) {
      sensorData.data[i] = 2550.f;
    }

    switch (sensorData.usActuatorMode) {
    case SensorData::leftToLeft: {
      float leftUsValue = float(((SimRobotCore2::SensorPort*)leftUsSensor)->getValue().floatValue * scale);
      sensorData.data[SensorData::us] = addJitter(leftUsValue);
      break;
    }
    case SensorData::leftToRight: {
      float centerLeftUsValue = float(((SimRobotCore2::SensorPort*)centerLeftUsSensor)->getValue().floatValue * scale);
      sensorData.data[SensorData::us] = addJitter(centerLeftUsValue);
      break;
    }
    case SensorData::rightToLeft: {
      float centerRightUsValue = float(((SimRobotCore2::SensorPort*)centerRightUsSensor)->getValue().floatValue * scale);
      sensorData.data[SensorData::us] = addJitter(centerRightUsValue);
      break;
    }
    case SensorData::rightToRight: {
      float rightUsValue = float(((SimRobotCore2::SensorPort*)rightUsSensor)->getValue().floatValue * scale);
      sensorData.data[SensorData::us] = addJitter(rightUsValue);
      break;
    }
    case SensorData::bothToSame: {
      float leftUsValue = float(((SimRobotCore2::SensorPort*)leftUsSensor)->getValue().floatValue * scale);
      float rightUsValue = float(((SimRobotCore2::SensorPort*)rightUsSensor)->getValue().floatValue * scale);
      sensorData.data[SensorData::usL] = addJitter(leftUsValue);
      sensorData.data[SensorData::us] = addJitter(rightUsValue);
      break;
    }
    case SensorData::bothToOther: {
      float centerLeftUsValue = float(((SimRobotCore2::SensorPort*)centerLeftUsSensor)->getValue().floatValue * scale);
      float centerRightUsValue = float(((SimRobotCore2::SensorPort*)centerRightUsSensor)->getValue().floatValue * scale);
      sensorData.data[SensorData::usL] = addJitter(centerLeftUsValue);
      sensorData.data[SensorData::usR] = addJitter(centerRightUsValue);
      break;
    }
    default:
      ASSERT(false);
    }
    sensorData.usTimeStamp = sensorData.timeStamp;
  }
}

void Oracle::getOrientationData(const SensorData& sensorData, OrientationData& orientationData) {
  ASSERT(robot);
  getPose3D(robot, robotPose3D);
  const Pose3D offset(lastRobotPose3D.invert().conc(robotPose3D));
  orientationData.rotation = robotPose3D.rotation;
  float timeScale = 1.f / (float(sensorData.timeStamp - lastTimeStamp) * 0.001f);
  orientationData.velocity.x = float(offset.translation.y * timeScale);
  orientationData.velocity.y = -float(offset.translation.x * timeScale);
  orientationData.velocity.z = float(offset.translation.z * timeScale);
  lastRobotPose3D = robotPose3D;
  lastTimeStamp = sensorData.timeStamp;
}

void Oracle::moveRobot(const Vector3<>& pos, const Vector3<>& rot, bool changeRotation) {
  ASSERT(robot);

  Vector3<> position = pos * 0.001f;
  if (changeRotation) {
    RotationMatrix rotation(rot);
    float rotation2[3][3];
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        rotation2[i][j] = rotation[i][j];
      }
    }
    ((SimRobotCore2::Body*)robot)->move(&position.x, rotation2);
  } else {
    ((SimRobotCore2::Body*)robot)->move(&position.x);
  }
}

void Oracle::enablePhysics(bool enable) {
  ((SimRobotCore2::Body*)robot)->enablePhysics(enable);
}

void Oracle::moveBall(const Vector3<>& pos, bool resetDynamics) {
  Vector3<> position = pos * 0.001f;
  ((SimRobotCore2::Body*)ball)->move(&position.x);
  if (resetDynamics) {
    ((SimRobotCore2::Body*)ball)->resetDynamics();
  }
}

Vector2<> Oracle::getPosition(SimRobot::Object* obj) {
  const float* position = ((SimRobotCore2::Body*)obj)->getPosition();
  return Vector2<>(position[0], position[1]) * 1000.f;
}

Vector3<> Oracle::getPosition3D(SimRobot::Object* obj) {
  const float* position = ((SimRobotCore2::Body*)obj)->getPosition();
  return Vector3<>(position[0], position[1], position[2]) * 1000.f;
}

void Oracle::mirrorPose(Pose2D& pose) {
  pose = Pose2D(pi) + pose;
}

bool Oracle::getPose2D(SimRobot::Object* obj, Pose2D& pose2D) const {
  float position[3];
  float rot3d[3][3];
  ((SimRobotCore2::Body*)obj)->getPose(position, rot3d);

  pose2D.translation = Vector2<>(position[0], position[1]) * 1000.f;

  // compute z-rotation

  /*
  Vector3<> d = Vector3<>(-rot3d[0][2], -rot3d[1][2], rot3d[2][2]);
  Vector3<> g = Vector3<>(0, 0, 1.f) ^ d;
  float w = atan2(sqrt(d.x * d.x + d.y * d.y), d.z);
  RotationMatrix withoutZ(g, w);
  RotationMatrix zOnly = RotationMatrix(Vector3<>(rot3d[0][0], rot3d[0][1], rot3d[0][2]), Vector3<>(rot3d[1][0], rot3d[1][1],
  rot3d[1][2]), Vector3<>(rot3d[2][0], rot3d[2][1], rot3d[2][2])) * withoutZ.invert();
  pose2D.rotation = atan2(zOnly.c0.y, zOnly.c0.x);
  */

  // (this is an optimized version of the code above)
  float x = rot3d[1][2], y = -rot3d[0][2];
  const float z = rot3d[2][2];
  const float gLenSqr = x * x + y * y;
  const float gLen = std::sqrt(gLenSqr);
  const float wLen = std::sqrt(gLenSqr + z * z);
  if (gLen != 0.f) {
    x /= gLen;
    y /= gLen;
  }
  const float si = -gLen / wLen, co = z / wLen;
  const float v = 1 - co;
  const float d0x = x * x * v + co;
  const float d0y = x * y * v;
  const float d0z = -y * si;
  const float c0x = rot3d[0][0] * d0x + rot3d[1][0] * d0y + rot3d[2][0] * d0z;
  const float c0y = rot3d[0][1] * d0x + rot3d[1][1] * d0y + rot3d[2][1] * d0z;
  pose2D.rotation = std::atan2(c0y, c0x);
  return rot3d[2][2] >= 0.3;
}

void Oracle::getPose3D(SimRobot::Object* obj, Pose3D& pose3D) const {
  float rotation[3][3];
  ((SimRobotCore2::Body*)obj)->getPose(&pose3D.translation.x, rotation);

  pose3D.translation *= 1000.f;
  pose3D.rotation.c0.x = rotation[0][0];
  pose3D.rotation.c0.y = rotation[0][1];
  pose3D.rotation.c0.z = rotation[0][2];
  pose3D.rotation.c1.x = rotation[1][0];
  pose3D.rotation.c1.y = rotation[1][1];
  pose3D.rotation.c1.z = rotation[1][2];
  pose3D.rotation.c2.x = rotation[2][0];
  pose3D.rotation.c2.y = rotation[2][1];
  pose3D.rotation.c2.z = rotation[2][2];
}

float Oracle::addJitter(float value) {
  if (value >= 1400.f) { // nothing was measured
    return value;
  } else {
    value = std::max(260.f, value);
    float randValue = (float)(0.1f * (rand() % 100) + -0.1f * (rand() % 100));

    if (rand() % 100 <= 10) {
      return 2200;
    } else {
      return value + randValue;
    }
  }
}

bool Oracle::isHomeTeam(SimRobot::Object* obj) {
  QString robotNumberString(obj->getFullName());
  int pos = robotNumberString.lastIndexOf('.');
  robotNumberString.remove(0, pos + 6);
  return robotNumberString.toInt() < 6;
}

int Oracle::getNumber(const SimRobot::Object* obj) {
  QString robotNumberString = obj->getFullName();
  int pos = robotNumberString.lastIndexOf('.');
  robotNumberString.remove(0, pos + 6);
  return robotNumberString.toInt();
}

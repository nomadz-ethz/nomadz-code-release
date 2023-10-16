/**
 * @file RobotPose.h
 *
 * The file contains the definition of the class RobotPose.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Core/Debugging/DebugDrawings.h"
#include "Core/Math/Pose2D.h"
#include "Core/Math/Matrix3x3.h"
#include "Core/Streams/AutoStreamable.h"
#include "Tools/SelfLocator/UKFSample.h"
#include "Representations/Perception/LineAnalysis.h"

#include <vector>

/**
 * @class RobotPose
 * The pose of the robot with additional information
 */
#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/robot_pose.hpp"
#endif
STREAMABLE_DECLARE_WITH_POSE2D(RobotPose)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/robot_pose_sample.hpp"
#endif
STREAMABLE_DECLARE(RobotPoseSample)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/robot_pose_samples.hpp"
#endif
STREAMABLE_DECLARE(RobotPoseSamples)

STREAMABLE_WITH_POSE2D(RobotPose, {
public:
  enum { unknownDeviation = 100000 };

  /** Assignment operator for Pose2D objects
   * @param other A Pose2D object
   * @return A reference to the object after the assignment
   */
  RobotPose& operator=(const Pose2D& other) {
    (Pose2D&)* this = other;
    // validity and co are not set
    return *this;
  }

  /** Draws the robot pose in the color of the team to the field view*/
  void draw(bool teamRed);

  void drawOnImage(
    const CameraMatrix&, const CameraInfo&, const FieldDimensions&, const ColorRGBA color = ColorClasses::black),
    FIELD_WRAPPER(float,
                  0,
                  nomadz_msgs::msg::RobotPose::validity,
                  validity), /**< The validity of the robot pose. (0 = invalid, 1 = perfect) */
    FIELD_WRAPPER(
      float, unknownDeviation, nomadz_msgs::msg::RobotPose::deviation, deviation), /**< The deviation of the robot pose. */
    FIELD_WRAPPER_DEFAULT(Matrix3x3<>,
                          nomadz_msgs::msg::RobotPose::covariance,
                          covariance), /**< The covariance matrix of the estimated robot pose. */
    FIELD_WRAPPER(unsigned,
                  0,
                  nomadz_msgs::msg::RobotPose::time_when_last_valid,
                  timeWhenLastValid), /**< The time stamp when validity was higher than SelfLocator.validityThreshold */
    FIELD_WRAPPER(unsigned,
                  0,
                  nomadz_msgs::msg::RobotPose::time_since_last_valid,
                  timeSinceLastValid), /**< The time (ms) since validity crossed under SelfLocator.validityThreshold */
    FIELD_WRAPPER(unsigned int,
                  0,
                  nomadz_msgs::msg::RobotPose::time_last_sensor_update,
                  timeLastSensorUpdate), /**< Used for SLAM (in behavior) */
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::RobotPose::corrected, corrected),
    FIELD_WRAPPER(unsigned, 0, nomadz_msgs::msg::RobotPose::timestamp, timestamp), /**< Only used for ground state */
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::RobotPose::valid, valid),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::RobotPose::lost, lost),
});

/**
 * @class RobotPoseSamples
 *
 * A class that contains particle filter samples from SelfLocator
 */
STREAMABLE_ROS(RobotPoseSamples, {
public:
  /** A single obstacle */
  STREAMABLE_ROS(RobotPoseSample, {
    public :
      /** Constructor */
      RobotPoseSample(float x, float y, float angle, float validity, float deviation),
    FIELD_WRAPPER_DEFAULT(float, nomadz_msgs::msg::RobotPoseSample::x, x),
    FIELD_WRAPPER_DEFAULT(float, nomadz_msgs::msg::RobotPoseSample::y, y),
    FIELD_WRAPPER_DEFAULT(float, nomadz_msgs::msg::RobotPoseSample::angle, angle),
    FIELD_WRAPPER_DEFAULT(float, nomadz_msgs::msg::RobotPoseSample::validity, validity),
    FIELD_WRAPPER_DEFAULT(float, nomadz_msgs::msg::RobotPoseSample::deviation, deviation),
  });

  /** Function for drawing */
  // void draw() const;

  void draw() const, /*added just to make it compile*/

    /** A list of obstacles */
    FIELD_WRAPPER_DEFAULT(std::vector<RobotPoseSample>, nomadz_msgs::msg::RobotPoseSamples::samples, samples),
});

/**
 * @class GroundTruthRobotPose
 * The same as the RobotPose, but - in general - provided by an external
 * source that has ground truth quality
 */
STREAMABLE_ALIAS(GroundTruthRobotPose, RobotPose, {
public:
  /** Draws the robot pose to the field view*/
  void draw() const;
});

/**
 * @struct RobotPoseAfterPreview
 * The same as the RobotPose, including
 * the preview phase of Dortmund Walking Engine.
 */
STREAMABLE_ALIAS(RobotPoseAfterPreview, RobotPose, { void draw() const; });

STREAMABLE_ALIAS(FixedOdometryRobotPose, RobotPose, {});

/**
 * @class RobotPoseCompressed
 * A compressed version of RobotPose used in team communication
 */
STREAMABLE_DECLARE_LEGACY(RobotPoseCompressed)
STREAMABLE_LEGACY(RobotPoseCompressed, {
public:
  RobotPoseCompressed(const RobotPose& robotPose);
  operator RobotPose() const,

    (Vector2<short>)translation, (char)rotation, (unsigned char)validity, (float)deviation,
});

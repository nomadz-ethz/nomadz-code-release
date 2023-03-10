/**
 * @file TemplateGenerator.h
 *
 * This file declares a submodule that generates robot positions from percepts.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 * and <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/LineAnalysis.h"
#include "Core/RingBuffer.h"
#include "vector"

/**
 * @class TemplateGenerator
 *
 * A module for computing poses from percepts
 */
class TemplateGenerator {
private:
  const GoalPercept& theGoalPercept;
  const LineAnalysis& theLineAnalysis;
  const FrameInfo& theFrameInfo;
  const FieldDimensions& theFieldDimensions;
  const OdometryData& theOdometryData;

  /**
   * @class SampleTemplate
   * Robot position generated from perceptions
   */
  class SampleTemplate : public Pose2D {
  public:
    /** Constructor */
    SampleTemplate() : Pose2D(), timestamp(0) {}

    /** Constructor */
    SampleTemplate(const Pose2D& pose) : Pose2D(pose), timestamp(0) {}

    /** Timestamp of visual input for construction of this template */
    unsigned timestamp;

    /** Indicates whether the sample is just a random position or has been computed from (a) seen goal post(s) */
    enum Origin { GOAL, RANDOM } origin;
  };

  class FullGoal {
  public:
    Vector2<> seenLeftPosition;
    Vector2<> realLeftPosition;
    Vector2<> seenRightPosition;
    Vector2<> realRightPosition;
    int timestamp;
    Pose2D odometry;
  };

  class KnownGoalpost {
  public:
    Vector2<> seenPosition;
    Vector2<> realPosition;
    int timestamp;
    Pose2D odometry;
    bool centerCircleSeen;
    Vector2<> centerCircleSeenPosition;
  };

  class UnknownGoalpost { // NOLINT
  public:
    Vector2<> seenPosition;
    Vector2<> realPositions[2];
    int realPositionGuess;
    int timestamp;
    Pose2D odometry;
    bool centerCircleSeen;
    Vector2<> centerCircleSeenPosition;
  };

  enum { MAX_PERCEPTS = 10 };
  RingBuffer<FullGoal, MAX_PERCEPTS> fullGoals;
  RingBuffer<KnownGoalpost, MAX_PERCEPTS> knownGoalposts;
  RingBuffer<UnknownGoalpost, MAX_PERCEPTS> unknownGoalposts;
  Vector2<> realPostPositions[GoalPost::numOfPositions];
  std::vector<Pose2D> walkInPositions;
  unsigned int nextWalkInTemplateNumber;

  template <typename T> void removeOldPercepts(RingBuffer<T, MAX_PERCEPTS>& buffer, int templateMaxKeepTime);

  /**
   * Generates a new sample by using the perceptions of both posts of a goal
   * @param goal The goal
   * @return A samples; calling function has to check the timestamp of the generated sample to determine its validity
   */
  SampleTemplate generateTemplateFromFullGoal(const FullGoal& goal, float standardDeviationGoalpostSamplingDistance) const;

  SampleTemplate generateTemplateFromPositionAndCenterCircle(const Vector2<>& posSeen,
                                                             const Vector2<>& circlePosSeen,
                                                             const Vector2<>& posReal,
                                                             const Pose2D& postOdometry,
                                                             float standardDeviationGoalpostSamplingDistance) const;

  SampleTemplate generateTemplateFromPosition(const Vector2<>& posSeen,
                                              const Vector2<>& posReal,
                                              const Pose2D& postOdometry,
                                              float standardDeviationGoalpostSamplingDistance) const;

  SampleTemplate generateTemplate(const Pose2D& lastPose, float standardDeviationGoalpostSamplingDistance) const;

  bool isMirrorCloser(const Pose2D& currentPose, const Pose2D& lastPose, float useRotationThreshold) const;

  bool
  halfChangeNeeded(const Pose2D& pose, const Pose2D& odometry, const Vector2<>& seenPost, const Vector2<>& realPost) const;

public:
  enum ForceHalf { OWN_HALF, OPPONENT_HALF, CONSIDER_POSE, RANDOM_HALF };

  TemplateGenerator(const GoalPercept& goalPercept,
                    const LineAnalysis& lineAnalysis,
                    const FrameInfo& frameInfo,
                    const FieldDimensions& fieldDimensions,
                    const OdometryData& odometryData);

  /** Empty all buffers. */
  void init();

  /** Buffers current goal perceptions. */
  void bufferNewPerceptions(const Pose2D& robotPose,
                            float standardDeviationGoalpostSamplingDistance,
                            float templateUnknownPostAssumptionMaxDistance,
                            int templateMaxKeepTime);

  Pose2D getTemplate(ForceHalf forceHalf,
                     float mirrorLikelihood,
                     float useRotationThreshold,
                     float standardDeviationGoalpostSamplingDistance,
                     Pose2D robotPose,
                     const Pose2D& lastRobotPose) const;

  Pose2D getTemplateAtReenterPosition(int i) const;

  Pose2D getTemplateAtWalkInPosition(Pose2D initPose);

  Pose2D getTemplateAtManualPlacementPosition(int robotNumber);

  bool templatesAvailable() const;

  void draw();
};

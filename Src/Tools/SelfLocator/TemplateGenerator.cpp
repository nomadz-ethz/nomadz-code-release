/**
 * @file TemplateGenerator.cpp
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

#include "TemplateGenerator.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"
#include "Core/Math/Probabilistics.h"

TemplateGenerator::TemplateGenerator(const GoalPercept& goalPercept,
                                     const LineAnalysis& lineAnalysis,
                                     const FrameInfo& frameInfo,
                                     const FieldDimensions& fieldDimensions,
                                     const OdometryData& odometryData)
    : theGoalPercept(goalPercept), theLineAnalysis(lineAnalysis), theFrameInfo(frameInfo),
      theFieldDimensions(fieldDimensions), theOdometryData(odometryData), nextWalkInTemplateNumber(0) {}

void TemplateGenerator::init() {
  realPostPositions[GoalPost::IS_LEFT] = Vector2<>(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal);
  realPostPositions[GoalPost::IS_RIGHT] =
    Vector2<>(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal);
  fullGoals.init();
  knownGoalposts.init();
  unknownGoalposts.init();
  walkInPositions.clear();
  //  walkInPositions.push_back( Pose2D(fromDegrees(-120.f),-1000.f, theFieldDimensions.yPosLeftSideline) );
  //  walkInPositions.push_back( Pose2D(fromDegrees(-110.f),-2000.f, theFieldDimensions.yPosLeftSideline) );
  //  walkInPositions.push_back( Pose2D(fromDegrees(-100.f),-3000.f, theFieldDimensions.yPosLeftSideline) );
  //  walkInPositions.push_back( Pose2D(fromDegrees( 120.f),-1000.f, theFieldDimensions.yPosRightSideline) );
  //  walkInPositions.push_back( Pose2D(fromDegrees( 110.f),-2000.f, theFieldDimensions.yPosRightSideline) );
  //  walkInPositions.push_back( Pose2D(fromDegrees( 100.f),-3000.f, theFieldDimensions.yPosRightSideline) );
  walkInPositions.push_back(Pose2D(fromDegrees(0.f), 0.f, 0.f));
  walkInPositions.push_back(Pose2D(fromDegrees(0.f), 0.f, 0.f));
  walkInPositions.push_back(Pose2D(fromDegrees(0.f), 0.f, 0.f));
  walkInPositions.push_back(Pose2D(fromDegrees(0.f), 0.f, 0.f));
  walkInPositions.push_back(Pose2D(fromDegrees(0.f), 0.f, 0.f));
  walkInPositions.push_back(Pose2D(fromDegrees(0.f), 0.f, 0.f));
}

Pose2D TemplateGenerator::getTemplate(TemplateGenerator::ForceHalf forceHalf,
                                      float mirrorLikelihood,
                                      float useRotationThreshold,
                                      float standardDeviationGoalpostSamplingDistance,
                                      Pose2D robotPose,
                                      const Pose2D& lastRobotPose) const {
  if (forceHalf == TemplateGenerator::CONSIDER_POSE) {
    // Check mirroring of robotPose: (TODO: think about some potential problems)
    if (randomFloat() < mirrorLikelihood) {
      robotPose = Pose2D(pi) + robotPose;
    }
    TemplateGenerator::SampleTemplate t = generateTemplate(lastRobotPose, standardDeviationGoalpostSamplingDistance);
    if (isMirrorCloser(t, lastRobotPose, useRotationThreshold)) {
      return Pose2D(pi) + t;
    } else {
      return t;
    }
  } else {
    TemplateGenerator::SampleTemplate t = generateTemplate(lastRobotPose, standardDeviationGoalpostSamplingDistance);
    if (forceHalf == TemplateGenerator::OWN_HALF) {
      if (t.translation.x <= 0.f) {
        return t;
      } else {
        return Pose2D(pi) + t;
      }
    } else if (forceHalf == TemplateGenerator::OPPONENT_HALF) {
      if (t.translation.x > 0.f) {
        return t;
      } else {
        return Pose2D(pi) + t;
      }
    } else // forceHalf == RANDOM_HALF
    {
      if (t.origin == TemplateGenerator::SampleTemplate::RANDOM) {
        return t;
      } else {
        if (randomFloat() < 0.5) {
          return t;
        } else {
          return Pose2D(pi) + t;
        }
      }
    }
  }
}

void TemplateGenerator::bufferNewPerceptions(const Pose2D& robotPose,
                                             float standardDeviationGoalpostSamplingDistance,
                                             float templateUnknownPostAssumptionMaxDistance,
                                             int templateMaxKeepTime) {
  if (!theGoalPercept.goalPosts.empty()) {
    const Vector2<>& positionOnField1(theGoalPercept.goalPosts[0].positionOnField);

    // Buffer data generated from GoalPercept:
    if (theGoalPercept.goalPosts.size() == 2) {
      const Vector2<>& positionOnField2(theGoalPercept.goalPosts[1].positionOnField);
      FullGoal newFullGoal;
      const bool leftBeforeRight = theGoalPercept.goalPosts[0].position == GoalPost::IS_LEFT;
      newFullGoal.realLeftPosition = realPostPositions[GoalPost::IS_LEFT];
      newFullGoal.realRightPosition = realPostPositions[GoalPost::IS_RIGHT];
      newFullGoal.seenLeftPosition = leftBeforeRight ? positionOnField1 : positionOnField2;
      newFullGoal.seenRightPosition = leftBeforeRight ? positionOnField2 : positionOnField1;
      newFullGoal.timestamp = theFrameInfo.time;
      newFullGoal.odometry = theOdometryData;

      // Before adding, check if templates can be generated from this perception
      SampleTemplate checkTemplate = generateTemplateFromFullGoal(newFullGoal, standardDeviationGoalpostSamplingDistance);
      if (checkTemplate.timestamp) {
        fullGoals.add(newFullGoal);
      }
    } else {
      const GoalPost& post = theGoalPercept.goalPosts[0];
      if (post.position != GoalPost::IS_UNKNOWN) {
        // We might currently see a single goal post with known side (but not a complete goal)
        KnownGoalpost newPost;
        newPost.realPosition = realPostPositions[post.position];
        newPost.seenPosition = positionOnField1;
        newPost.timestamp = theFrameInfo.time;
        newPost.odometry = theOdometryData;
        newPost.centerCircleSeen = theLineAnalysis.circle.found;
        if (newPost.centerCircleSeen) {
          newPost.centerCircleSeenPosition = theLineAnalysis.circle.pos;
        }
        knownGoalposts.add(newPost);
      } else {
        // Maybe we have seen some goalpost of which we do not know the side:
        UnknownGoalpost newPost;
        newPost.realPositions[0] = realPostPositions[GoalPost::IS_LEFT];
        newPost.realPositions[1] = realPostPositions[GoalPost::IS_RIGHT];
        newPost.seenPosition = positionOnField1;
        newPost.timestamp = theFrameInfo.time;
        newPost.odometry = theOdometryData;
        newPost.centerCircleSeen = theLineAnalysis.circle.found;
        if (newPost.centerCircleSeen) {
          newPost.centerCircleSeenPosition = theLineAnalysis.circle.pos;
        }
        // Try to make a guess about the real position of the goal post
        if (positionOnField1.abs() < templateUnknownPostAssumptionMaxDistance) {
          Vector2<> postWorld = robotPose * positionOnField1;
          if (postWorld.x < 0) { // Always assume opponent half here
            postWorld = Pose2D(pi) * postWorld;
          }
          const float d0 = (postWorld - newPost.realPositions[0]).abs();
          const float d1 = (postWorld - newPost.realPositions[1]).abs();
          newPost.realPositionGuess = d0 < d1 ? 0 : 1;
        } else {
          newPost.realPositionGuess = -1;
        }
        unknownGoalposts.add(newPost);
      }
    }
  }

  // If there are still some too old percepts after adding new ones -> delete them:
  removeOldPercepts(fullGoals, templateMaxKeepTime);
  removeOldPercepts(knownGoalposts, templateMaxKeepTime);
  removeOldPercepts(unknownGoalposts, templateMaxKeepTime);
}

template <typename T>
void TemplateGenerator::removeOldPercepts(RingBuffer<T, MAX_PERCEPTS>& buffer, int templateMaxKeepTime) {
  while (buffer.getNumberOfEntries()) {
    T& oldestElement = buffer[buffer.getNumberOfEntries() - 1];
    if (theFrameInfo.getTimeSince(oldestElement.timestamp) > templateMaxKeepTime) {
      buffer.removeFirst();
    } else {
      break;
    }
  }
}

TemplateGenerator::SampleTemplate
TemplateGenerator::generateTemplate(const Pose2D& lastPose, float standardDeviationGoalpostSamplingDistance) const {
  TemplateGenerator::SampleTemplate newTemplate;
  // Current solution: Prefer to construct templates from full goals only:
  if (fullGoals.getNumberOfEntries()) {
    const FullGoal& goal = fullGoals[rand() % fullGoals.getNumberOfEntries()];
    newTemplate = generateTemplateFromFullGoal(goal, standardDeviationGoalpostSamplingDistance);
  } else if (knownGoalposts.getNumberOfEntries()) {
    const KnownGoalpost& goalPost = knownGoalposts[rand() % knownGoalposts.getNumberOfEntries()];
    if (goalPost.centerCircleSeen) {
      newTemplate = generateTemplateFromPositionAndCenterCircle(goalPost.seenPosition,
                                                                goalPost.centerCircleSeenPosition,
                                                                goalPost.realPosition,
                                                                goalPost.odometry,
                                                                standardDeviationGoalpostSamplingDistance);
    }
    if (newTemplate.timestamp == 0) {
      newTemplate = generateTemplateFromPosition(
        goalPost.seenPosition, goalPost.realPosition, goalPost.odometry, standardDeviationGoalpostSamplingDistance);
    }
  } else if (unknownGoalposts.getNumberOfEntries()) {
    const UnknownGoalpost& goalPost = unknownGoalposts[rand() % unknownGoalposts.getNumberOfEntries()];
    const Vector2<>& realPosition = goalPost.realPositionGuess == -1 ? goalPost.realPositions[rand() % 2]
                                                                     : goalPost.realPositions[goalPost.realPositionGuess];
    if (goalPost.centerCircleSeen) {
      newTemplate = generateTemplateFromPositionAndCenterCircle(goalPost.seenPosition,
                                                                goalPost.centerCircleSeenPosition,
                                                                realPosition,
                                                                goalPost.odometry,
                                                                standardDeviationGoalpostSamplingDistance);
    }
    if (newTemplate.timestamp == 0) {
      newTemplate = generateTemplateFromPosition(
        goalPost.seenPosition, realPosition, goalPost.odometry, standardDeviationGoalpostSamplingDistance);
    }
  }
  if (newTemplate.timestamp == 0) // In some cases, no proper sample is generated, return the last position and go ahead...
  {
    newTemplate = lastPose;
    newTemplate.origin = TemplateGenerator::SampleTemplate::RANDOM; // Needs to be changed in the future...
  }
  return newTemplate;
}

bool TemplateGenerator::isMirrorCloser(const Pose2D& currentPose, const Pose2D& lastPose, float useRotationThreshold) const {
  const Vector2<>& translation = currentPose.translation;
  Vector2<> rotationWeight(std::max(useRotationThreshold - std::min(translation.abs(), lastPose.translation.abs()), 0.f), 0);
  Vector2<> rotation = Pose2D(currentPose.rotation) * rotationWeight;
  Vector2<> lastRotation = Pose2D(lastPose.rotation) * rotationWeight;

  return (lastPose.translation - translation).abs() + (lastRotation - rotation).abs() >
         (lastPose.translation + translation).abs() + (lastRotation + rotation).abs();
}

bool TemplateGenerator::templatesAvailable() const {
  const int sumOfTemplates =
    fullGoals.getNumberOfEntries() + knownGoalposts.getNumberOfEntries() + unknownGoalposts.getNumberOfEntries();
  return sumOfTemplates > 0;
}

TemplateGenerator::SampleTemplate
TemplateGenerator::generateTemplateFromFullGoal(const FullGoal& goal,
                                                float standardDeviationGoalpostSamplingDistance) const {
  TemplateGenerator::SampleTemplate newTemplate;
  Pose2D odometryOffset = theOdometryData - goal.odometry;
  float leftPostDist = goal.seenLeftPosition.abs();
  float leftDistUncertainty = sampleTriangularDistribution(standardDeviationGoalpostSamplingDistance);
  if (leftPostDist + leftDistUncertainty > standardDeviationGoalpostSamplingDistance) {
    leftPostDist += leftDistUncertainty;
  }
  float rightPostDist = goal.seenRightPosition.abs();
  float rightDistUncertainty = sampleTriangularDistribution(standardDeviationGoalpostSamplingDistance);
  if (rightPostDist + rightDistUncertainty > standardDeviationGoalpostSamplingDistance) {
    rightPostDist += rightDistUncertainty;
  }
  Geometry::Circle c1(goal.realLeftPosition, leftPostDist + theFieldDimensions.goalPostRadius);
  Geometry::Circle c2(goal.realRightPosition, rightPostDist + theFieldDimensions.goalPostRadius);
  // If there are intersections, take the first one that is in the field:
  Vector2<> p1, p2;
  int result = Geometry::getIntersectionOfCircles(c1, c2, p1, p2);
  if (result) {
    if (theFieldDimensions.isInsideCarpet(p1)) {
      float origAngle = (goal.realLeftPosition - p1).angle();
      float observedAngle = goal.seenLeftPosition.angle();
      Pose2D templatePose(origAngle - observedAngle, p1);
      templatePose += odometryOffset;
      newTemplate = templatePose;
      newTemplate.timestamp = theFrameInfo.time;
      newTemplate.origin = TemplateGenerator::SampleTemplate::GOAL;
    } else if (theFieldDimensions.isInsideCarpet(p2)) {
      float origAngle = (goal.realLeftPosition - p2).angle();
      float observedAngle = goal.seenLeftPosition.angle();
      Pose2D templatePose(origAngle - observedAngle, p2);
      templatePose += odometryOffset;
      newTemplate = templatePose;
      newTemplate.timestamp = theFrameInfo.time;
      newTemplate.origin = TemplateGenerator::SampleTemplate::GOAL;
    }
  }
  // The else case is omitted, calling function has to check the timestamp of the generated sample
  return newTemplate;
}

TemplateGenerator::SampleTemplate
TemplateGenerator::generateTemplateFromPositionAndCenterCircle(const Vector2<>& posSeen,
                                                               const Vector2<>& circlePosSeen,
                                                               const Vector2<>& posReal,
                                                               const Pose2D& postOdometry,
                                                               float standardDeviationGoalpostSamplingDistance) const {
  TemplateGenerator::SampleTemplate newTemplate;
  Pose2D odometryOffset = theOdometryData - postOdometry;
  float postDist = posSeen.abs();
  float postDistUncertainty = sampleTriangularDistribution(standardDeviationGoalpostSamplingDistance);
  if (postDist + postDistUncertainty > standardDeviationGoalpostSamplingDistance) {
    postDist += postDistUncertainty;
  }
  float circleDist = circlePosSeen.abs();
  float circleDistUncertainty = sampleTriangularDistribution(
    standardDeviationGoalpostSamplingDistance); // No special uncertainty for center circle available
  if (circleDist + circleDistUncertainty > standardDeviationGoalpostSamplingDistance) {
    circleDist += circleDistUncertainty;
  }
  Geometry::Circle c1(posReal, postDist + theFieldDimensions.goalPostRadius);
  Geometry::Circle c2(Vector2<>(0.0f, 0.0f), circleDist);
  // If there are intersections, take the first one that is in the field:
  Vector2<> p1, p2;
  int result = Geometry::getIntersectionOfCircles(c1, c2, p1, p2);
  if (result) {
    if (theFieldDimensions.isInsideCarpet(p1)) {
      float origAngle = (posReal - p1).angle();
      float observedAngle = posSeen.angle();
      Pose2D templatePose(origAngle - observedAngle, p1);
      templatePose += odometryOffset;
      newTemplate = templatePose;
      newTemplate.timestamp = theFrameInfo.time;
      newTemplate.origin = TemplateGenerator::SampleTemplate::GOAL;
    } else if (theFieldDimensions.isInsideCarpet(p2)) {
      float origAngle = (posReal - p2).angle();
      float observedAngle = posSeen.angle();
      Pose2D templatePose(origAngle - observedAngle, p2);
      templatePose += odometryOffset;
      newTemplate = templatePose;
      newTemplate.timestamp = theFrameInfo.time;
      newTemplate.origin = TemplateGenerator::SampleTemplate::GOAL;
    }
  }
  // The else case is omitted, calling function has to check the timestamp of the generated sample
  return newTemplate;
}

TemplateGenerator::SampleTemplate
TemplateGenerator::generateTemplateFromPosition(const Vector2<>& posSeen,
                                                const Vector2<>& posReal,
                                                const Pose2D& postOdometry,
                                                float standardDeviationGoalpostSamplingDistance) const {
  TemplateGenerator::SampleTemplate newTemplate;
  float r = posSeen.abs() + theFieldDimensions.goalPostRadius;
  float distUncertainty = sampleTriangularDistribution(standardDeviationGoalpostSamplingDistance);
  if (r + distUncertainty > standardDeviationGoalpostSamplingDistance) {
    r += distUncertainty;
  }
  Vector2<> realPosition = posReal;
  float minY = std::max(posReal.y - r, static_cast<float>(theFieldDimensions.yPosRightFieldBorder));
  float maxY = std::min(posReal.y + r, static_cast<float>(theFieldDimensions.yPosLeftFieldBorder));
  Vector2<> p;
  p.y = minY + randomFloat() * (maxY - minY);
  float xOffset(std::sqrt(sqr(r) - sqr(p.y - posReal.y)));
  p.x = posReal.x;
  p.x += (p.x > 0) ? -xOffset : xOffset;
  if (theFieldDimensions.isInsideCarpet(p)) {
    float origAngle = (realPosition - p).angle();
    float observedAngle = posSeen.angle();
    Pose2D templatePose(origAngle - observedAngle, p);
    Pose2D odometryOffset = theOdometryData - postOdometry;
    templatePose += odometryOffset;
    newTemplate = templatePose;
    newTemplate.timestamp = theFrameInfo.time;
    newTemplate.origin = TemplateGenerator::SampleTemplate::GOAL;
  }
  return newTemplate;
}

Pose2D TemplateGenerator::getTemplateAtReenterPosition(int i) const {
  const float interval = 500.f; // mm
  const std::vector<Pose2D> reenterPositions = {
    {-pi_2, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosLeftSideline},
    {pi_2, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosRightSideline},
    {-pi_2, theFieldDimensions.xPosOwnPenaltyMark + interval, theFieldDimensions.yPosLeftSideline},
    {pi_2, theFieldDimensions.xPosOwnPenaltyMark + interval, theFieldDimensions.yPosRightSideline},
    {-pi_2, theFieldDimensions.xPosOwnPenaltyMark + 2 * interval, theFieldDimensions.yPosLeftSideline},
    {pi_2, theFieldDimensions.xPosOwnPenaltyMark + 2 * interval, theFieldDimensions.yPosRightSideline},
  };
  return reenterPositions[i % reenterPositions.size()];
}

Pose2D TemplateGenerator::getTemplateAtWalkInPosition(Pose2D initPose) {
  //  Pose2D result = walkInPositions[nextWalkInTemplateNumber];
  //  nextWalkInTemplateNumber = (nextWalkInTemplateNumber + 1) % walkInPositions.size();
  Pose2D result = initPose;
  return result;
}

Pose2D TemplateGenerator::getTemplateAtManualPlacementPosition(int robotNumber) {
  // Goalie
  if (robotNumber == 1) {
    return Pose2D(0.f, theFieldDimensions.xPosOwnGroundline, 0.f);
  } else {
    float x = theFieldDimensions.xPosOwnPenaltyArea + 100.f;
    float minY = theFieldDimensions.yPosRightSideline + 750.f;
    float y = minY + randomFloat() * (2 * std::abs(minY));
    return Pose2D(0.f, x, y);
  }
}

void TemplateGenerator::draw() {
  for (int i = 0; i < fullGoals.getNumberOfEntries(); ++i) {
    FullGoal& goal = fullGoals[i];
    Pose2D odometryOffset = goal.odometry - theOdometryData;
    Vector2<> leftPost = odometryOffset * goal.seenLeftPosition;
    Vector2<> rightPost = odometryOffset * goal.seenRightPosition;
    LINE("module:SelfLocator:templates",
         leftPost.x,
         leftPost.y,
         rightPost.x,
         rightPost.y,
         50,
         Drawings::ps_solid,
         ColorRGBA(140, 140, 255));
    CIRCLE("module:SelfLocator:templates",
           leftPost.x,
           leftPost.y,
           100,
           20,
           Drawings::ps_solid,
           ColorRGBA(0, 0, 0),
           Drawings::bs_solid,
           ColorRGBA(140, 140, 255));
    CIRCLE("module:SelfLocator:templates",
           rightPost.x,
           rightPost.y,
           100,
           20,
           Drawings::ps_solid,
           ColorRGBA(0, 0, 0),
           Drawings::bs_solid,
           ColorRGBA(140, 140, 255));
  }
  for (int i = 0; i < knownGoalposts.getNumberOfEntries(); ++i) {
    KnownGoalpost& post = knownGoalposts[i];
    Pose2D odometryOffset = post.odometry - theOdometryData;
    Vector2<> postPos = odometryOffset * post.seenPosition;
    CIRCLE("module:SelfLocator:templates",
           postPos.x,
           postPos.y,
           100,
           20,
           Drawings::ps_solid,
           ColorRGBA(140, 140, 255),
           Drawings::bs_solid,
           ColorRGBA(140, 140, 255));
    CIRCLE("module:SelfLocator:templates",
           postPos.x,
           postPos.y,
           200,
           20,
           Drawings::ps_solid,
           ColorRGBA(0, 0, 0),
           Drawings::bs_null,
           ColorRGBA(140, 140, 255));
  }
}

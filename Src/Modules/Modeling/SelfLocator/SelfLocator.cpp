/**
 * @file SelfLocator.cpp
 *
 * Implements a class that performs self-localization
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 */

#include "SelfLocator.h"
#include "Core/Global.h"
#include "Core/Math/Probabilistics.h"
#include "Tools/Geometry/Transformations.h"
#include "Representations/Infrastructure/Image.h"

#include <algorithm>
#include <numeric>

#include <iostream>
#include <fstream>

#include <Eigen/Eigenvalues> //simonzi
#include <Eigen/Dense>

using namespace std;

static std::string toString(const Vector<3, float>& mean) {
  return "r = " + std::to_string(std::round(mean.z * 180.f / M_PI * 10.f) / 10.f) + " deg, " +
         "x = " + std::to_string(std::round(mean.x)) + " mm, y = " + std::to_string(std::round(mean.y)) + " mm";
}

SelfLocator::SelfLocator()
    : fieldModel(theFieldDimensions, theCameraMatrix),
      sampleGenerator(theGoalPercept, theLineAnalysis, theFrameInfo, theFieldDimensions, theOdometryData),
      mirrorLikelihood(0.f), lastPenalty(-1), lastGameState(-1), timeOfLastFall(0), lastTimeWithoutArmContact(0),
      lastTimeFarGoalSeen(0), timePenaltyWasLeft(-1) {
  // Set up subcomponent for sample resetting
  sampleGenerator.init();

  // Create sample set with samples at the typical walk-in positions
  samples = new SampleSet<UKFSample>(numberOfSamples);

  // Hard coded init positions (see config file)
  initWalkInPositions();

  // Init counters for particle replacement
  for (int i = 0; i < samples->size(); i++) {
    badParticleCounter.push_back(0);
  }
  noCorrectionCounter = 0;
  lostStateCounter = 0;
}

SelfLocator::~SelfLocator() {
  delete samples;
}

void SelfLocator::update(RobotPose& robotPose) {
  /* Initialize variable(s) */
  sampleSetHasBeenResetted = false;

  /* Keep the sample generator up to date:
   *  - new goal perceptions are buffered
   *  - old goal perceptions are deleted
   *  - the pose is needed for making guesses about unknown goal post assignment
   */
  Pose2D propagatedRobotPose = robotPose + theOdometer.odometryOffset;
  sampleGenerator.bufferNewPerceptions(propagatedRobotPose,
                                       standardDeviationGoalpostSamplingDistance,
                                       templateUnknownPostAssumptionMaxDistance,
                                       templateMaxKeepTime);

  /* Modify sample set according to certain changes of the game state:
   *  - Reset mirror in SET
   *  - Handling of penalty positions
   *  - ...
   */
  handleGameStateChanges(propagatedRobotPose);

  /* Move all samples according to the current odometry.
   */
  if (applyMotionUpdate) {
    motionUpdate();
  }

  // LINE LOCATOR
  std::vector<Pose2D> lineLocatorSuggestions;
  if (applyLineLocator && theCameraMatrix.isValid) {

    // Nudge the samples (particles) around
    STOP_TIME_ON_REQUEST("module:SelfLocator:hypotheses", { lineLocatorSuggestions = runLineLocator(robotPose); });

    // Draw thick circle around current robot pose
    CIRCLE("representation:RobotPose",
           robotPose.translation.x,
           robotPose.translation.y,
           250,
           25,
           Drawings::ps_solid,
           ColorRGBA(lastHypothesis.color),
           Drawings::bs_null,
           ColorClasses::none);
  }

  DECLARE_DEBUG_DRAWING("module:SelfLocator:debug", "drawingOnField"); // Draws FoV & visible field lines
  DECLARE_DEBUG_DRAWING("module:SelfLocator:fov", "drawingOnField");   // Draws FoV & visible field lines
  DECLARE_DEBUG_DRAWING("module:SelfLocator:povs",
                        "drawingOnField"); // Draws FoV & visible field lines from PoV of each particle
  DECLARE_DEBUG_DRAWING("module:SelfLocator:povsOverlay",
                        "drawingOnField"); // Text labels that should appear above everything else
  DECLARE_DEBUG_DRAWING("module:SelfLocator:correcting", "drawingOnField"); // Draws untransformed observations
  DECLARE_DEBUG_DRAWING("module:SelfLocator:matches", "drawingOnField"); // Draws matches between field lines & observations
  DECLARE_DEBUG_DRAWING("module:SelfLocator:corrected", "drawingOnField"); // Draws field lines corrected by transform
  DECLARE_PLOT("module:SelfLocator:validity");
  DECLARE_PLOT("module:SelfLocator:validity0");
  DECLARE_PLOT("module:SelfLocator:validity1");
  DECLARE_PLOT("module:SelfLocator:validity2");
  DECLARE_PLOT("module:SelfLocator:validity3");
  DECLARE_PLOT("module:SelfLocator:validity4");
  DECLARE_PLOT("module:SelfLocator:validity5");

  if (applyPoseTracking && theCameraMatrix.isValid && (theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT)) {

    STOP_TIME_ON_REQUEST("module:SelfLocator:poseTracking", {
      MODIFY("representation:RobotPose", robotPose);

      const bool isUpper = (theCameraInfo.camera == CameraInfo::upper);

      std::vector<Segment> observations; // Field lines we currently see (both cameras)
      if (!isUpper)
        observations.clear();

      // Import observations
      for (int i = 0, n = theLineAnalysis.rawSegs.size(); i < n; ++i) {
        const LineAnalysis::LineSegment& observation = theLineAnalysis.rawSegs[i];
        observations.push_back(Segment(observation.alpha, observation.d, observation.p1, observation.p2));
      }

      // Only continue processing on upper camera
      if (isUpper && !observations.empty()) {

        // Add each LineLocator suggestion better than the current worst sample
        for (Pose2D& suggestion : lineLocatorSuggestions) {
          std::vector<Prior> priorLines;
          STOP_TIME_ON_REQUEST("trackPose:importPriors", { relativePriors(suggestion, priorLines); });

          UKFSample& worstSample = getLeastValidSample();
          const float worstValidity = worstSample.computeValidity();
          const float suggestionValidity = matchRatio(priorLines, observations, suggestion, {});
          if (suggestionValidity > 0 && suggestionValidity >= worstValidity) {
            worstSample.init(suggestion, defaultPoseDeviation, suggestionValidity);
          }
        }

        // Sensor update for each particle
        for (int i = 0; i < samples->size(); ++i) {
          auto& sample = samples->at(i);

          const auto& mean = sample.mean;
          Pose2D samplePose(mean.z, mean.x, mean.y);

          if (drawWhichSample == i)
            std::cout << "Sample " << i << std::endl;
          if (drawWhichSample == i)
            std::cout << "    mean = " << mean.z << ", " << mean.x << ", " << mean.y << std::endl;
          if (drawWhichSample == i)
            std::cout << "    variance = " << sample.getVarianceWeighting() << std::endl;

          std::vector<Prior> samplePriorLines;
          STOP_TIME_ON_REQUEST("trackPose:importPriors", { relativePriors(samplePose, samplePriorLines); });

          Pose2D localCorrection; // local relative to sample mean
          Matrix3x3f localCovar;
          bool bounded;
          STOP_TIME_ON_REQUEST("trackPose:iterativeEstimator", {
            bounded = iterativeEstimator(
              samplePose, samplePriorLines, observations, localCorrection, localCovar, (i == drawWhichSample));
          });

          if (drawWhichSample == i)
            std::cout << "    bounded = " << bounded << std::endl;
          if (drawWhichSample == i)
            std::cout << "    localCorrection = " << localCorrection.rotation << ", " << localCorrection.translation.x
                      << ", " << localCorrection.translation.y << std::endl;

          const float ratio = matchRatio(samplePriorLines, observations, samplePose, localCorrection);
          if (drawWhichSample == i)
            std::cout << "    ratio = " << ratio << std::endl;

          if (bounded && (ratio > minInlierRatio) && localCorrection != Pose2D()) {

            const Matrix2x2<> rotMatrix(std::cos(-samplePose.rotation),
                                        std::sin(-samplePose.rotation), // row 1
                                        -std::sin(-samplePose.rotation),
                                        std::cos(-samplePose.rotation));

            Matrix2x2<> localCovarT(localCovar[0][0], localCovar[0][1], localCovar[1][0], localCovar[1][1]);
            if (i == drawWhichSample)
              COVARIANCE2D("module:SelfLocator:corrected", localCovarT, localCorrection.translation);

            Pose2D correctedPose = localCorrection;
            correctedPose.translation.rotate(samplePose.rotation);
            correctedPose.translation += samplePose.translation;
            correctedPose.rotation += samplePose.rotation;
            if (drawWhichSample == i)
              std::cout << "    correctedPose = " << correctedPose.rotation << ", " << correctedPose.translation.x << ", "
                        << correctedPose.translation.y << std::endl;

            Matrix2x2<> covarT = rotMatrix * localCovarT * rotMatrix.transpose();
            Matrix3x3f covar(Vector<3, float>(covarT[0][0], covarT[1][0], 0.f), // column 1
                             Vector<3, float>(covarT[0][1], covarT[1][1], 0.f),
                             Vector<3, float>(0.f, 0.f, localCovar[2][2]));

            if (drawWhichSample == i)
              std::cout << "    validity = " << sample.computeValidity();
            sample.updateByTracker(
              Vector<3, float>(correctedPose.translation.x, correctedPose.translation.y, correctedPose.rotation),
              covar,
              ratio,
              true);
            if (drawWhichSample == i)
              std::cout << " --> " << sample.computeValidity() << std::endl;

          } else {
            if (drawWhichSample == i)
              std::cout << "    validity = " << sample.computeValidity();
            if (theOdometer.odometryOffset == Pose2D()) {
              // If standing, make sample increasingly invalid
              sample.updateByTracker(Vector<3, float>(), Matrix3x3f(), 0.f, false);
            } else {
              // If moving, don't drastically decrease the validity
              sample.updateByTracker(Vector<3, float>(), Matrix3x3f(), sample.computeValidity() / 2.f, false);
            }
            if (drawWhichSample == i)
              std::cout << " --> " << sample.computeValidity() << std::endl;
          }
        }
      }
    };);
  }

  /* Finally, update internal variables, debug and draw stuff.
   */
  // To adjust robot pose manually (for testing)
  if (resetRobotPose) {
    Pose2D pose;
    pose.rotation = robotPose.rotation;
    pose.translation.x = robotPose.translation.x;
    pose.translation.y = robotPose.translation.y;
    for (int i = 0; i < samples->size(); ++i) {
      samples->at(i).init(pose, defaultPoseDeviation);
    }
  }

  // Invalidate any samples that get really close to theRobotPose's mirror position.
  // (TODO Sounds hacky? Probably is! Replace with actual sources of real outside information, like things from team
  // communication.)
  if (applyFlipKilling) {
    STOP_TIME_ON_REQUEST("module:SelfLocator:killFlips", { killFlips(); });
  }

  // Compute final robot pose
  if (applyResampling) {
    STOP_TIME_ON_REQUEST("module:SelfLocator:resample", { resample(); });
  }

  STOP_TIME_ON_REQUEST("module:SelfLocator:computeModel", { computeModel(robotPose); });

  const bool isStrikerOrSupporter = (theRobotInfo.number == 4 || theRobotInfo.number == 5);

  // Validate position with keeper's ball detection, if on exact opposite site: flip
  const Vector2<> opponentGoalLocation = {theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal};
  if (theTeamMateData.isFullyActive[1] && theBallModel.timeSinceLastSeen < 3000 &&
      theCombinedWorldModel.timeSinceBallOthersLastSeenPlayer[1] < 5000 &&
      (opponentGoalLocation - theRobotPose.translation).abs() < 3250.f && isStrikerOrSupporter) {

    const Vector2<> keeperBallPos = theCombinedWorldModel.ballStateOthers.position;
    const Vector2<> keeperBallPosMirrored = -keeperBallPos;
    const Vector2<> ownBallPos = theRobotPose * theBallModel.estimate.position;

    if ((ownBallPos - keeperBallPosMirrored).abs() < 500.f) {
      int ballAlsoSeenBy = -1;
      for (int i = 2; i < TeamMateData::numOfPlayers; i++) {
        if (i == theRobotInfo.number) {
          continue;
        }
        if (theTeamMateData.isFullyActive[i] &&
            theFrameInfo.getTimeSince(theTeamMateData.ballModels[i].timeWhenLastSeen) < 3000) {
          const Vector2<> otherBallPos = theTeamMateData.robotPoses[i] * theTeamMateData.ballModels[i].lastPerception;
          if ((ownBallPos - otherBallPos).abs() < 1000.f) {
            ballAlsoSeenBy = i;
          }
        }
      }
      if (ballAlsoSeenBy == -1) {
        for (int i = 0; i < samples->size(); ++i) {
          samples->at(i).mirror();
        }

        robotPose = Pose2D(pi) + robotPose;
        std::cout << "Emergency flip! ownBallPos: (" << ownBallPos.x << ", " << ownBallPos.y << ")" << std::endl;
      } else {
        std::cout << "Emergency flip aborted: robot " << ballAlsoSeenBy << " also sees it with "
                  << "ownBallPos: (" << ownBallPos.x << ", " << ownBallPos.y << ")" << std::endl;
      }
    }
  }

  DEBUG_RESPONSE_ONCE("module:SelfLocator:flip", {
    for (int i = 0; i < numberOfSamples; ++i) {
      samples->at(i).mirror();
    }
  });

  DEBUG_RESPONSE("module:SelfLocator:templates_only", {
    if (sampleGenerator.templatesAvailable()) {
      for (int i = 0; i < numberOfSamples; ++i) {
        UKFSample newSample;
        if (theOwnSideModel.stillInOwnSide)
          newSample.init(sampleGenerator.getTemplate(TemplateGenerator::OWN_HALF,
                                                     mirrorLikelihood,
                                                     useRotationThreshold,
                                                     standardDeviationGoalpostSamplingDistance,
                                                     robotPose,
                                                     lastRobotPose),
                         defaultPoseDeviation);
        else
          newSample.init(sampleGenerator.getTemplate(TemplateGenerator::CONSIDER_POSE,
                                                     mirrorLikelihood,
                                                     useRotationThreshold,
                                                     standardDeviationGoalpostSamplingDistance,
                                                     robotPose,
                                                     lastRobotPose),
                         defaultPoseDeviation);
        samples->at(i) = newSample;
      }
    }
  });

  lastRobotPose = robotPose;
  MODIFY("representation:RobotPose", robotPose);

  draw();

  EXECUTE_ONLY_IN_DEBUG(robotPose.draw(theOwnTeamInfo.teamColor != TEAM_BLUE););
  EXECUTE_ONLY_IN_DEBUG(robotPose.drawOnImage(theCameraMatrix, theCameraInfo, theFieldDimensions, ColorClasses::black););
  DECLARE_DEBUG_DRAWING("origin:Odometry", "drawingOnField", {
    Pose2D origin = robotPose + theOdometryData.invert();
    ORIGIN("origin:Odometry", origin.translation.x, origin.translation.y, origin.rotation);
  });
}

void SelfLocator::update(RobotPoseSamples& robotPoseSamples) {
  robotPoseSamples.samples.clear();
  for (int i = 0; i < samples->size(); ++i) {
    UKFSample& sample = samples->at(i);
    const Matrix3x3f& cov = sample.getCov();
    robotPoseSamples.samples.push_back(RobotPoseSamples::RobotPoseSample(
      sample.mean.x, sample.mean.y, sample.mean.z, sample.validity, sqrt(std::max(cov[0].x, cov[1].y))));
  }
}

void SelfLocator::update(SideConfidence& sideConfidence) {
  // mirror flag is not computed by this module
  sideConfidence.mirror = false;

  // Compute confidence value
  // Not playing -> sideConfidence 100%
  if (theGameInfo.state != STATE_PLAYING || theRobotInfo.penalty != PENALTY_NONE) {
    sideConfidence.sideConfidence = sideConfidenceConfident;
  }
  // Playing but on the safe side -> sideConfidence 100%
  else if (theOwnSideModel.stillInOwnSide) {
    sideConfidence.sideConfidence = sideConfidenceConfident;
  }
  // Leaving the safe side -> sideConfidence max 95%
  else {
    float confidence = sideConfidenceAlmostConfident;
    int numberOfMirroredSamples = 0;
    const int numberOfSamples = samples->size();
    for (int i = 0; i < numberOfSamples; ++i) {
      if (samples->at(i).isMirrored()) {
        ++numberOfMirroredSamples;
      }
    }
    if (numberOfMirroredSamples > numberOfSamples / 2) {
      numberOfMirroredSamples = numberOfSamples / 2; // FIXME
    }
    sideConfidence.sideConfidence = confidence * (1.f - static_cast<float>(numberOfMirroredSamples) / (numberOfSamples / 2));
  }

  // Set confidence state:
  if (sideConfidence.sideConfidence == sideConfidenceConfident) {
    sideConfidence.confidenceState = SideConfidence::CONFIDENT;
  } else if (sideConfidence.sideConfidence == sideConfidenceAlmostConfident) {
    sideConfidence.confidenceState = SideConfidence::ALMOST_CONFIDENT;
  } else if (sideConfidence.sideConfidence > sideConfidenceConfused) {
    sideConfidence.confidenceState = SideConfidence::UNSURE;
  } else {
    sideConfidence.confidenceState = SideConfidence::CONFUSED;
  }
}

void SelfLocator::motionUpdate() {
  const float transNoise = translationNoise;
  const float rotNoise = rotationNoise;
  const float transX = theOdometer.odometryOffset.translation.x;
  const float transY = theOdometer.odometryOffset.translation.y;
  const float dist = theOdometer.odometryOffset.translation.abs();
  const float angle = abs(theOdometer.odometryOffset.rotation);

  // precalculate rotational error that has to be adapted to all samples
  const float rotError = max(rotNoise, max(dist * movedDistWeight, angle * movedAngleWeight));

  // precalculate translational error that has to be adapted to all samples
  const float transXError = max(transNoise, max(abs(transX * majorDirTransWeight), abs(transY * minorDirTransWeight)));
  const float transYError = max(transNoise, max(abs(transY * majorDirTransWeight), abs(transX * minorDirTransWeight)));

  // update samples
  for (int i = 0; i < numberOfSamples; ++i) {
    const Vector2<> transOffset((transX - transXError) + (2 * transXError) * randomFloat(),
                                (transY - transYError) + (2 * transYError) * randomFloat());
    const float rotationOffset = theOdometer.odometryOffset.rotation + (randomFloat() * 2 - 1) * rotError;

    samples->at(i).motionUpdate(
      Pose2D(rotationOffset, transOffset), filterProcessDeviation, odometryDeviation, odometryRotationDeviation);
  }
}

void SelfLocator::handleGameStateChanges(const Pose2D& propagatedRobotPose) {
  // We are in a penalty shootout!
  if (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT) {
    // penalty shoot: if game state switched to playing reset samples to start position
    if ((lastGameState != STATE_PLAYING && theGameInfo.state == STATE_PLAYING) ||
        (lastPenalty != PENALTY_NONE && theRobotInfo.penalty == PENALTY_NONE)) {
      if (theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber) {
        // striker pose (1 meter behind the penalty spot, looking towards opponent goal)
        for (int i = 0; i < samples->size(); ++i) {
          samples->at(i).init(
            Pose2D(0.f, theFieldDimensions.xPosOpponentPenaltyMark - 1000.f, 0.f), defaultPoseDeviation, 1.f);
        }
        sampleSetHasBeenResetted = true;
      } else {
        // goalie pose (in the center of the goal, looking towards the field's center)
        for (int i = 0; i < samples->size(); ++i) {
          samples->at(i).init(Pose2D(0.f, theFieldDimensions.xPosOwnGroundline, 0.f), defaultPoseDeviation, 1.f);
        }
        sampleSetHasBeenResetted = true;
      }
    }
  }
  // If a penalty is over, reset samples to reenter positions
  else if (theOwnSideModel.returnFromGameControllerPenalty || theOwnSideModel.returnFromManualPenalty) {
    // Except when penalized for motion in set
    if (lastPenalty != PENALTY_SPL_ILLEGAL_MOTION_IN_SET) {
      for (int i = 0; i < samples->size(); ++i) {
        Pose2D pose = sampleGenerator.getTemplateAtReenterPosition(i);
        samples->at(i).init(pose, defaultPoseDeviation);
      }
      sampleSetHasBeenResetted = true;
      timePenaltyWasLeft = theFrameInfo.time;
    }
  }
  // Normal game is about to start: We start on the sidelines looking at our goal: (this is for checking in TeamCom)
  else if (lastGameState != STATE_INITIAL && theGameInfo.state == STATE_INITIAL) {
    initWalkInPositions();
    sampleSetHasBeenResetted = true;
  }
  // Normal game really starts: We start on the sidelines looking at our goal: (this is for actual setup)
  else if (lastGameState == STATE_INITIAL && theGameInfo.state == STATE_READY) {
    initWalkInPositions();
    sampleSetHasBeenResetted = true;
  }
  // In SET state, a robot cannot be mirrored => reset flags
  if (lastGameState == STATE_SET && theGameInfo.state != STATE_SET && theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT) {
    for (int i = 0; i < samples->size(); ++i) {
      if (samples->at(i).mean.x > 0) {
        samples->at(i).mirror();
      }
    }
  }
  // Update members
  lastGameState = theGameInfo.state;
  lastPenalty = theRobotInfo.penalty;
}

UKFSample& SelfLocator::getMostValidSample() const {
  UKFSample* returnSample = &(samples->at(0));
  float maxValidity = -1.f;
  float minVariance = 0.f; // Initial value does not matter
  for (int i = 0; i < numberOfSamples; ++i) {
    if (samples->at(i).isMirrored()) {
      continue;
    }
    const float val = samples->at(i).validity;
    if (val > maxValidity) {
      maxValidity = val;
      minVariance = samples->at(i).getVarianceWeighting();
      returnSample = &(samples->at(i));
    } else if (val == maxValidity) {
      float variance = samples->at(i).getVarianceWeighting();
      if (variance < minVariance) {
        maxValidity = val;
        minVariance = variance;
        returnSample = &(samples->at(i));
      }
    }
  }
  return *returnSample;
}

UKFSample& SelfLocator::getLeastValidSample() const {
  UKFSample* returnSample = &(samples->at(0));
  float minValidity = 1.f;
  float maxVariance = INFINITY; // Initial value does not matter
  for (int i = 0; i < numberOfSamples; ++i) {
    if (samples->at(i).isMirrored()) {
      continue;
    }
    const float val = samples->at(i).validity;
    if (val < minValidity) {
      minValidity = val;
      maxVariance = samples->at(i).getVarianceWeighting();
      returnSample = &(samples->at(i));
    } else if (val == minValidity) {
      float variance = samples->at(i).getVarianceWeighting();
      if (variance > maxVariance) {
        minValidity = val;
        maxVariance = variance;
        returnSample = &(samples->at(i));
      }
    }
  }
  return *returnSample;
}

// Rotation weight is calculated using distance of pose closer to the origin
float SelfLocator::poseDistance(const Pose2D& me, const Pose2D& sample) const {
  const float distToCenter = std::min(me.translation.abs(), sample.translation.abs());
  return (me.translation - sample.translation).abs() +
         std::max(0.f,
                  (centerDangerRadius - distToCenter) / centerDangerRadius * 2000.f *
                    fabs(angleDifference(sample.rotation, me.rotation)));
}

bool SelfLocator::isMirrorCloser(const Pose2D& me, const Pose2D& sample) const {
  return poseDistance(Pose2D(pi) + sample, me) < poseDistance(sample, me);
}

// reads: theLineLocalization.hypotheses, samples, currentPose (to break symmetry)
// writes: lastHypothesis, samples
// returns: a vector of hypotheses that the LineLocator suggests should be kept track of
std::vector<Pose2D> SelfLocator::runLineLocator(const Pose2D& currentPose) {
  typedef LineLocalization::Hypothesis Hypothesis;

  std::vector<Hypothesis> hypotheses = theLineLocalization.hypotheses;

  DECLARE_DEBUG_DRAWING("module:SelfLocator:hscores", "drawingOnField");

  std::vector<Pose2D> suggestions;

  if (hypotheses.size() > 0) {
    // Line locator update available:
    // For each particle, perform an update
    lastHypothesis = hypotheses[0];

    bool existingSampleUpdated = false;
    for (int i = 0; i < samples->size(); ++i) {
      float minDist = INFINITY;
      Pose2D bestPose;
      for (size_t j = 0; j < hypotheses.size(); ++j) {
        // Coming back from penalty -> discard all hypotheses on other side for 5 Seconds
        if (timePenaltyWasLeft > 0 && theFrameInfo.time - timePenaltyWasLeft < 5000 &&
            hypotheses[j].pose.translation.x > 0) {
          continue;
        }

        // HACK? Ignore all hypotheses on wrong side (i.e. own side) if in penalty shootout
        if (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT && theOwnTeamInfo.teamColor == theGameInfo.kickingTeam &&
            hypotheses[j].pose.translation.x < 0) {
          continue;
        }

        // If in initial state, ignore all hypotheses on wrong side (i.e. opponent side)
        if (theGameInfo.state == STATE_INITIAL && hypotheses[j].pose.translation.x > 0) {
          continue;
        }

        // If in set state, ignore all hypotheses on wrong side (i.e. opponent side)
        if (theGameInfo.state == STATE_SET && hypotheses[j].pose.translation.x > 0) {
          continue;
        }

        // If penalized, ignore all hypotheses on wrong side (i.e. opponent side)
        if (theRobotInfo.penalty != PENALTY_NONE && hypotheses[j].pose.translation.x > 0) {
          continue;
        }

        const float dist = poseDistance(hypotheses[j].pose, samples->at(i).getPose());
        DRAWTEXT("module:SelfLocator:hscores",
                 samples->at(i).mean.x,
                 samples->at(i).mean.y,
                 12,
                 ColorClasses::white,
                 (round(dist * 10.f) / 10.f));

        if (dist < minDist) {
          bestPose = hypotheses[j].pose;
          minDist = dist;
        }
      }

      ASSERT(minDist >= 0.f);

      if (correctionWithinBounds(samples->at(i).getPose(), bestPose)) {
        samples->at(i).lineLocatorUpdate(bestPose,
                                         minDist * minDist * deviationPerDistance * deviationPerDistance,
                                         minDist * minDist * deviationPerDistance * deviationPerDistance,
                                         deviationR * deviationR);
        existingSampleUpdated = true;
      }
    }

    if (!existingSampleUpdated) {
      float minDist = INFINITY;
      Pose2D bestPose;
      for (int j = 0, nj = hypotheses.size(); j < nj; ++j) {
        const float dist = poseDistance(hypotheses[j].pose, currentPose);
        if (dist < minDist) {
          minDist = dist;
          bestPose = hypotheses[j].pose;
        }
      }

      suggestions.push_back(bestPose);
    }
  }

  return suggestions;
}

// Starting from correction (input), tries to iteratively improve it
// Returns a pose correction based on new observations, or an empty pose if no estimate possible
// The correction is in the robot frame
bool SelfLocator::iterativeEstimator(const Pose2D& robotPose,
                                     const std::vector<Prior>& priors,
                                     const std::vector<Segment>& observations,
                                     Pose2D& correction,
                                     Matrix3x3f& covar,
                                     bool debug) const {

  // Field view: draw initial observations
  if (debug) {
    COMPLEX_DRAWING("module:SelfLocator:correcting", {
      for (auto i = observations.begin(); i != observations.end(); ++i) {
        LINE("module:SelfLocator:correcting",
             i->p1.x,
             i->p1.y,
             i->p2.x,
             i->p2.y,
             5,
             Drawings::ps_solid,
             ColorRGBA(0, 255, 255));
      }

      CIRCLE("module:SelfLocator:correcting",
             0,
             0,
             maxTransCorr,
             5,
             Drawings::ps_solid,
             ColorClasses::blue,
             Drawings::bs_null,
             ColorClasses::none);
      LINE("module:SelfLocator:correcting",
           0,
           0,
           maxTransCorr * cos(-maxRotCorr),
           maxTransCorr * sin(-maxRotCorr),
           5,
           Drawings::ps_solid,
           ColorClasses::blue);
      LINE("module:SelfLocator:correcting",
           0,
           0,
           maxTransCorr * cos(maxRotCorr),
           maxTransCorr * sin(maxRotCorr),
           5,
           Drawings::ps_solid,
           ColorClasses::blue);
    });
  }

  std::vector<Match> matches;
  std::vector<float> inlierRatios;
  std::vector<Match> inliers;
  std::vector<Pose2D> steps;

  correction = Pose2D();
  for (int t = 0; t < iterationsAll; ++t) {
    matches.clear();
    STOP_TIME_ON_REQUEST("trackPose:match",
                         { inlierRatios.push_back(matchObservations(priors, observations, matches, false)); });

    if (debug && t == 0) {
      drawMatches(matches);
    }

    STOP_TIME_ON_REQUEST("trackPose:transform", {
      Pose2D step;
      estimateTransform(matches, step, covar, correction);
      correction.rotation += step.rotation;
      correction.translation += step.translation;
      steps.push_back(step);
    });

    ColorRGBA color(128, 0, 0, std::min<unsigned char>(255, (unsigned char)(63 + 192 / iterationsAll * t)));
    if (debug) {
      drawPose(correction, color);
    }
    if (debug) {
      drawCorrected(observations, correction, color);
    }
    if (debug) {
      drawPriors(priors, ColorRGBA(32, 32, 32));
    }
  }

  for (int t = 0; t < iterationsInliers; ++t) {
    matches.clear();
    inliers.clear();
    STOP_TIME_ON_REQUEST("trackPose:match", {
      inlierRatios.push_back(matchObservations(priors, observations, matches, false, correction));
      copyInliers(matches, inliers);
    });

    if (debug && t == 0 && iterationsAll == 0) {
      drawMatches(matches);
    }

    STOP_TIME_ON_REQUEST("trackPose:transform", {
      Pose2D step;
      estimateTransform(inliers, step, covar, correction, (t == iterationsInliers - 1));
      correction.rotation += step.rotation;
      correction.translation += step.translation;
      steps.push_back(step);
    });

    ColorRGBA color =
      (t == iterationsInliers - 1)
        ? (correctionWithinBounds(correction) ? ColorRGBA(0, 0, 128) : ColorRGBA(128, 0, 0))
        : ColorRGBA(0, 0, 0, std::min<unsigned char>(255, (unsigned char)(63 + 192 / (iterationsInliers - 1) * t)));

    if (debug) {
      drawPose(correction, color);
    }
    if (debug) {
      drawCorrected(observations, correction, color);
    }
    if (debug) {
      drawPriors(priors, ColorRGBA(32, 32, 32));
    }
  }

  matches.clear();
  float ratio;
  STOP_TIME_ON_REQUEST("trackPose:match", {
    ratio = matchObservations(priors, observations, matches, false, correction);
    inlierRatios.push_back(ratio);
  });
  if (debug) {
    drawMatches(matches, correction);
  }

  // Print matching costs (good indicator of "fit")
  if (debug) {
    for (size_t i = 0; i < inlierRatios.size(); ++i) {
      if (i < steps.size()) {
        DRAWTEXT("module:SelfLocator:debug",
                 0,
                 -150 * i - 150,
                 9,
                 ColorRGBA(64, 0, 0),
                 (int)i << ". (" << round(steps[i].rotation * 180.f / M_PI * 10.f) / 10.f << " deg, "
                        << round(steps[i].translation.x) << " mm, " << round(steps[i].translation.y)
                        << " mm), inliers: " << round(inlierRatios[i] * 100.f * 10.f) / 10.f);
      } else {
        DRAWTEXT("module:SelfLocator:debug",
                 0,
                 -150 * i - 150,
                 9,
                 ColorRGBA(64, 0, 0),
                 (int)i << ". inliers: " << round(inlierRatios[i] * 100.f * 10.f) / 10.f);
      }
    }

    DRAWTEXT("module:SelfLocator:corrected",
             0,
             0,
             9,
             ColorClasses::black,
             "(" << std::round(correction.rotation * 180.f / M_PI * 10.f) / 10.f << " deg, "
                 << std::round(correction.translation.x) << " mm, " << std::round(correction.translation.y) << " mm)");
  }

  // Grow covariance matrix inversely proportional to inlier ratio
  // (Less inliers = more uncertain)
  covar /= (ratio + 0.01f);

  return correctionWithinBounds(correction);
}

bool SelfLocator::correctionWithinBounds(const Pose2D& original, const Pose2D& corrected) const {
  return (std::abs(original.rotation - corrected.rotation) <= maxRotCorr) &&
         ((original.translation - corrected.translation).abs() <= maxTransCorr);
}

bool SelfLocator::correctionWithinBounds(const Pose2D& correction) const {
  return (std::abs(correction.rotation) <= maxRotCorr) && (correction.translation.abs() <= maxTransCorr);
}

// priors: must be clipped to expected viewport
float SelfLocator::priorMatchRatio(const std::vector<Prior>& priors,
                                   const std::vector<Segment>& observations,
                                   const Pose2D preTransform) const {
  const float penaltyMarkRadius = theFieldDimensions.fieldLinesWidth;

  const Vector2<>& preTranslation = preTransform.translation;
  const float preRotation = preTransform.rotation;

  float inlierCosts = 0.f;
  float totalCosts = 0.f;

  // For each prior, find a matching observation
  for (const Prior& prior : priors) {

    float bestCost = INFINITY;
    Segment dummy;
    Segment& bestObs = dummy;

    // Don't try to match circles to observations
    // TODO Try to match circles to observations
    if (prior.type == Prior::Type::Circle) {
      continue;
    }

    const Segment& seg = prior.segment;

    const Vector2<>& j1 = seg.p1;
    const Vector2<>& j2 = seg.p2;
    const float ja = seg.alpha;
    const float jd = seg.d;
    const Vector2<> jv = j2 - j1;
    const float jl = jv.abs();

    for (const Segment& i : observations) {
      float cost = INFINITY;

      const Vector2<> i1 = Vector2<>(i.p1).rotate(preRotation) + preTranslation;
      const Vector2<> i2 = Vector2<>(i.p2).rotate(preRotation) + preTranslation;
      const float il = (i1 - i2).abs();

      const float u1 = i1.squareAbs();               // only an approximation of perspective-induced uncertainty
      const float w1 = 1.f / (u1 + 1000.f * 1000.f); // PARAM distanceSoftening
      const float u2 = i2.squareAbs();               // only an approximation of perspective-induced uncertainty
      const float w2 = 1.f / (u2 + 1000.f * 1000.f); // PARAM distanceSoftening

      if (il < 2.f * penaltyMarkRadius) {
        continue;
      }

      if (0.5f * (std::sqrt(u1) + std::sqrt(u2)) > 8000.f) { // PARAM maxDist
        continue;
      }

      // Squared perpendicular distances from observed segment endpoints to prior segment
      const float li1 = abs(i1.x * cos(ja) + i1.y * sin(ja) - jd);
      const float li2 = abs(i2.x * cos(ja) + i2.y * sin(ja) - jd);

      // Extra cost if observed segment endpoints are laterally far away from prior segment endpoints
      const float lat_i1 = std::max(0.f, ((i1.x - j1.x) * (-jv.x) + (i1.y - j1.y) * (-jv.y)) / jl) +
                           std::max(0.f, ((i1.x - j2.x) * (jv.x) + (i1.y - j2.y) * (jv.y)) / jl);
      const float lat_i2 = std::max(0.f, ((i2.x - j1.x) * (-jv.x) + (i2.y - j1.y) * (-jv.y)) / jl) +
                           std::max(0.f, ((i2.x - j2.x) * (jv.x) + (i2.y - j2.y) * (jv.y)) / jl);

      cost = std::sqrt(0.5f * (w1 * (li1 * li1 + 0.45f * lat_i1 * lat_i1) +
                               w2 * (li2 * li2 + 0.45f * lat_i2 * lat_i2))); // PARAM lateralCostWeight (hack? :/)

      // HACK Discourage matching long observations to short prior lines?
      if (il > 2.f * jl) {
        cost *= (il / jl - 1.f);
      }

      if (cost < bestCost) {
        bestCost = cost;
        bestObs = i;
      }
    }

    if (!std::isinf(bestCost)) {
      const bool isInlier = (bestCost <= maxMatchCost * 1e-2);
      if (isInlier) {
        inlierCosts += jl / (bestCost + 1.f);
      }
      totalCosts += jl / (bestCost + 1.f);
    }
  }

  return (totalCosts > 0.f) ? (inlierCosts / totalCosts) : 0.f;
}

// preTransform applies only to observations
// Returns ratio of inlier segment lengths over total segment lengths
float SelfLocator::matchObservations(const std::vector<Prior>& priors,
                                     const std::vector<Segment>& observations,
                                     std::vector<Match>& matches,
                                     bool inliersOnly,
                                     const Pose2D preTransform,
                                     bool output) const {

  using std::abs;
  using std::cos;
  using std::sin;

  const float penaltyMarkRadius = theFieldDimensions.fieldLinesWidth;

  const Vector2<>& preTranslation = preTransform.translation;
  const float& preRotation = preTransform.rotation;

  float inlierCosts = 0.f;
  float totalCosts = 0.f;

  // For each observation, find a matching prior that minimizes a cost
  // TODO Include line length difference in cost
  for (auto i = observations.begin(); i != observations.end(); ++i) {
    const Vector2<> i1 = Vector2<>(i->p1).rotate(preRotation) + preTranslation;
    const Vector2<> i2 = Vector2<>(i->p2).rotate(preRotation) + preTranslation;
    const float il = (i1 - i2).abs();

    const float u1 = i1.squareAbs();               // only an approximation of perspective-induced uncertainty
    const float w1 = 1.f / (u1 + 1000.f * 1000.f); // PARAM distanceSoftening
    const float u2 = i2.squareAbs();               // only an approximation of perspective-induced uncertainty
    const float w2 = 1.f / (u2 + 1000.f * 1000.f); // PARAM distanceSoftening

    if (il < 2.f * penaltyMarkRadius) {
      continue;
    }

    if (0.5f * (std::sqrt(u1) + std::sqrt(u2)) > 8000.f) { // PARAM maxDist
      continue;
    }

    float bestCost = INFINITY;
    Prior dummy;
    Prior& bestPrior = dummy;

    for (auto j = priors.begin(); j != priors.end(); ++j) {
      float cost = INFINITY;

      if (j->type == Prior::Type::Circle) {
        const Circle& prior = j->circle;

        const Vector2<>& jc = prior.c;
        const float jr = prior.r;

        if (il > M_PI * jr) {
          continue; // Ignore observations longer than circumference of semi-circle
        }

        const float li1 = abs((i1 - jc).abs() - jr);
        const float li2 = abs((i2 - jc).abs() - jr);
        const float li_mid = abs(((i1 + i2) * 0.5f - jc).abs() - jr);

        cost = std::sqrt(0.25f * (w1 * li1 * li1 + (w1 + w2) * li_mid * li_mid + w2 * li2 * li2));

      } else if (j->type == Prior::Type::Segment) {
        const Segment& prior = j->segment;

        const Vector2<>& j1 = prior.p1;
        const Vector2<>& j2 = prior.p2;
        const float ja = prior.alpha;
        const float jd = prior.d;
        const Vector2<> jv = j2 - j1;
        const float jl = jv.abs();

        // Squared perpendicular distances from observed segment endpoints to prior segment
        const float li1 = abs(i1.x * cos(ja) + i1.y * sin(ja) - jd);
        const float li2 = abs(i2.x * cos(ja) + i2.y * sin(ja) - jd);

        // Extra cost if observed segment endpoints are laterally far away from prior segment endpoints
        const float lat_i1 = std::max(0.f, ((i1.x - j1.x) * (-jv.x) + (i1.y - j1.y) * (-jv.y)) / jl) +
                             std::max(0.f, ((i1.x - j2.x) * (jv.x) + (i1.y - j2.y) * (jv.y)) / jl);
        const float lat_i2 = std::max(0.f, ((i2.x - j1.x) * (-jv.x) + (i2.y - j1.y) * (-jv.y)) / jl) +
                             std::max(0.f, ((i2.x - j2.x) * (jv.x) + (i2.y - j2.y) * (jv.y)) / jl);

        cost = std::sqrt(0.5f * (w1 * (li1 * li1 + 0.45f * lat_i1 * lat_i1) +
                                 w2 * (li2 * li2 + 0.45f * lat_i2 * lat_i2))); // PARAM lateralCostWeight (hack? :/)

        // HACK Discourage matching long observations to short prior lines?
        if (il > 2.f * jl) {
          cost *= (il / jl - 1.f);
        }
        // if (il > 5.f*jl && jl < 250.f) cost += (il-jl)*(il-jl)/sqdist_i;
      }

      if (cost < bestCost) {
        bestCost = cost;
        bestPrior = *j;
      }
    }

    if (!std::isinf(bestCost)) {
      const bool isInlier = (bestCost <= maxMatchCost * 1e-2);
      if (output && (!inliersOnly || isInlier)) {
        matches.push_back(Match(bestPrior, *i, bestCost, isInlier));
      }
      if (isInlier) {
        inlierCosts += il / (bestCost + 1.f);
      }
      totalCosts += il / (bestCost + 1.f);
    }
  }

  return (totalCosts > 0.f) ? (inlierCosts / totalCosts) : 0.f;
}

void SelfLocator::drawPov(const std::vector<Prior>& priors,
                          const std::vector<Segment>& observations,
                          const std::vector<Vector2<>>& fov,
                          const Pose2D transform) const {
  for (const auto& prior : priors) {
    if (prior.type == Prior::Type::Circle) {
      const Vector2<> c = transform * prior.circle.c;
      CIRCLE("module:SelfLocator:povs",
             c.x,
             c.y,
             prior.circle.r,
             20,
             Drawings::ps_dot,
             ColorRGBA(0, 0, 0),
             Drawings::bs_null,
             ColorClasses::none);
    } else if (prior.type == Prior::Type::Segment) {
      const Vector2<> p1 = transform * prior.segment.p1;
      const Vector2<> p2 = transform * prior.segment.p2;
      LINE("module:SelfLocator:povs", p1.x, p1.y, p2.x, p2.y, 20, Drawings::ps_dot, ColorRGBA(0, 0, 0));
    }
  }

  for (const auto& observation : observations) {
    const Vector2<> p1 = transform * observation.p1;
    const Vector2<> p2 = transform * observation.p2;
    LINE("module:SelfLocator:povs", p1.x, p1.y, p2.x, p2.y, 30, Drawings::ps_solid, ColorRGBA(127, 0, 0));
  }

  std::vector<Vector2<>> fov2 = fov;
  fov2.push_back(fov2[0]);
  for (auto i = fov2.begin(); i != fov2.end() - 1; ++i) {
    const Vector2<> p1 = transform * *i;
    const Vector2<> p2 = transform * *(i + 1);
    LINE("module:SelfLocator:povs", p1.x, p1.y, p2.x, p2.y, 10, Drawings::ps_solid, ColorRGBA(0, 0, 127));
  }
}

float SelfLocator::matchRatio(const std::vector<Prior>& priors,
                              const std::vector<Segment>& observations,
                              const Pose2D pose,
                              const Pose2D preTransform) const {

  std::vector<Match> dummy;
  const float observationRatio = matchObservations(priors, observations, dummy, true, preTransform, false);

  if (matchPriors) {
    const std::vector<Vector2<>> fov = Geometry::computeFovQuadrangle(theCameraMatrix, theCameraInfo);
    const Geometry::Circle sightRange = {{}, (priorClipRange < 0.f ? INFINITY : priorClipRange)};
    const std::vector<Prior> clippedPriors = clipPriors(priors, fov, sightRange);

    const float priorRatio = priorMatchRatio(clippedPriors, observations, preTransform);
    const float ratio = (observationRatio + priorRatio) * 0.5f;

    drawPov(clippedPriors, observations, fov, pose);
    DRAW_ROBOT_POSE("module:SelfLocator:povs", pose, ColorRGBA(0, int(255 * ratio), 0));
    DRAWTEXT("module:SelfLocator:povsOverlay",
             pose.translation.x,
             pose.translation.y,
             13,
             ColorClasses::red,
             round(observationRatio * 100.f) << "|" << round(priorRatio * 100.f));

    return ratio;

  } else {
    return observationRatio;
  }
}

// Returns average cost of inliers
float SelfLocator::copyInliers(const std::vector<Match>& matches, std::vector<Match>& inliers) const {
  float totalCost = 0.f;
  int inlierCount = 0;
  for (auto m = matches.begin(); m != matches.end(); ++m) {
    if (m->inlier) {
      inliers.push_back(*m);
      totalCost += m->cost;
      inlierCount++;
    }
  }
  return totalCost / inlierCount;
}

float SelfLocator::meanMatchCost(const std::vector<Match>& matches, bool inliersOnly) const {
  float total = 0.f;
  for (auto const& match : matches) {
    if (inliersOnly && !match.inlier) {
      continue;
    }
    total += match.cost;
  }
  return total / matches.size();
}

// Calculates translation & rotation that moves field lines towards observations
bool SelfLocator::estimateTransform(
  const std::vector<Match>& matches, Pose2D& correction, Matrix3x3f& covar, const Pose2D preTransform, bool debug) const {

  using std::cos;
  using std::sin;

  correction.rotation = 0.f;
  correction.translation.x = 0.f;
  correction.translation.y = 0.f;

  if (matches.size() == 0) {
    return false;
  }

  // Optimize both rotation & translation, but discard translation
  Eigen::Vector4f thetaBoth;
  Eigen::Matrix4f covarBoth;
  const bool fittedBoth = fitBoth(matches, thetaBoth, covarBoth, preTransform);

  if (fittedBoth) {
    correction.rotation = std::atan2(thetaBoth[1], thetaBoth[0]);

    // Constrain rotation within [-pi/2, pi/2)
    while (correction.rotation < -M_PI / 2) {
      correction.rotation += M_PI;
    }
    while (correction.rotation >= M_PI / 2) {
      correction.rotation -= M_PI;
    }
  }

  if (debug) {
    DRAWTEXT("module:SelfLocator:debug",
             0,
             150,
             9,
             ColorClasses::black,
             "fitBoth = [" << thetaBoth[0] << ", " << thetaBoth[1] << ", " << thetaBoth[2] << ", " << thetaBoth[3] << "] "
                           << (fittedBoth ? "fitted" : "failed")
                           << "; rot = " << round(correction.rotation * 180.f / M_PI * 10.f) / 10.f << " deg");
  }

  // If fitBoth was successful, l should be close to 1.0f
  const float l = std::sqrt(thetaBoth[0] * thetaBoth[0] + thetaBoth[1] * thetaBoth[1]);

  if (!fittedBoth || l < 0.01f) {
    // If fitBoth estimated rotation wrong, re-estimate rotation
    Eigen::Vector2f thetaRot;
    Eigen::Matrix2f covarRot;
    const bool fittedRot = fitRotation(matches, thetaRot, covarRot, preTransform);

    const float l2 = std::sqrt(thetaRot[0] * thetaRot[0] + thetaRot[1] * thetaRot[1]);
    if (fittedRot && std::abs(l2 - 1.f) < std::abs(l - 1.f)) {
      correction.rotation = std::atan2(thetaRot[1], thetaRot[0]);

      // Assume rotation within [-pi/2, pi/2)
      while (correction.rotation < -M_PI / 2) {
        correction.rotation += M_PI;
      }
      while (correction.rotation >= M_PI / 2) {
        correction.rotation -= M_PI;
      }
    }

    if (debug) {
      DRAWTEXT("module:SelfLocator:debug",
               0,
               300,
               9,
               ColorClasses::black,
               "fitRot = [" << thetaRot[0] << ", " << thetaRot[1] << "] " << (fittedRot ? "fitted" : "failed")
                            << "; rot = " << round(correction.rotation * 180.f / M_PI * 10.f) / 10.f << " deg");
    }
  }

  // Re-estimate translation using rotation from fitBoth
  Pose2D preTransform2 = preTransform;
  preTransform2.rotation += correction.rotation;

  Eigen::Vector2f thetaTrans;
  Eigen::Matrix2f covarTrans;
  const bool fittedTrans = fitTranslation(matches, thetaTrans, covarTrans, preTransform2);

  if (debug) {
    DRAWTEXT("module:SelfLocator:debug",
             0,
             450,
             9,
             ColorClasses::black,
             "fitTranslation = [" << thetaTrans[0] << ", " << thetaTrans[1] << "] " << (fittedTrans ? "fitted" : "failed")
                                  << "; rot = " << round(correction.rotation * 180.f / M_PI * 10.f) / 10.f << " deg");
  }

  // Output translation
  if (fittedTrans) {
    correction.translation.x = thetaTrans[0];
    correction.translation.y = thetaTrans[1];
    covar = Matrix3x3f(
      Vector<3, float>(
        covarScale * covarScale * covarTrans(0, 0), covarScale * covarScale * covarTrans(1, 0), 0.f), // column 1
      Vector<3, float>(covarScale * covarScale * covarTrans(0, 1), covarScale * covarScale * covarTrans(1, 1), 0.f),
      Vector<3, float>(0.f, 0.f, deviationR * deviationR));
  }

  return fittedTrans;
}

bool SelfLocator::fitBoth(const std::vector<Match>& matches,
                          Eigen::Vector4f& theta,
                          Eigen::Matrix4f& covar,
                          const Pose2D preTransform) const {

  using std::cos;
  using std::sin;

  const int M = matches.size();
  if (M == 0) {
    return false;
  }

  Eigen::Matrix<float, Eigen::Dynamic, 4> phi(M * 2 + 2, 4); // 2 costs for each match
  Eigen::Matrix<float, Eigen::Dynamic, 1> k(M * 2 + 2, 1);

  const float preCos = cos(preTransform.rotation);
  const float preSin = sin(preTransform.rotation);
  const float preX = preTransform.translation.x;
  const float preY = preTransform.translation.y;

  for (int i = 0; i < M; ++i) {
    const auto& observation = matches[i].observation;

    const Vector2<> p1 = Vector2<>(preCos * observation.p1.x - preSin * observation.p1.y + preX,
                                   preSin * observation.p1.x + preCos * observation.p1.y + preY);
    const Vector2<> p2 = Vector2<>(preCos * observation.p2.x - preSin * observation.p2.y + preX,
                                   preSin * observation.p2.x + preCos * observation.p2.y + preY);
    const float ol = (p1 - p2).abs();

    const float u1 =
      p1.squareAbs() / ol; // only an approximation of perspective-induced uncertainty; longer segments are more certain
    const float w1 = 1.f / (u1 + 1000.f); // PARAM distanceSoftening
    const float u2 =
      p2.squareAbs() / ol; // only an approximation of perspective-induced uncertainty; longer segments are more certain
    const float w2 = 1.f / (u2 + 1000.f); // PARAM distanceSoftening

    if (matches[i].prior.type == Prior::Type::Circle) {

      const auto& circle = matches[i].prior.circle;
      const Vector2<>& c = circle.c;
      const float r = circle.r;

      const Vector2<> v1 = p1 - c;
      const float v1l = v1.abs();
      const Vector2<> inter1 = v1 * (r / v1l - 1.f);

      const Vector2<> v2 = p2 - c;
      const float v2l = v2.abs();
      const Vector2<> inter2 = v2 * (r / v2l - 1.f);

      // Approximately fit translation to circle (don't ask me, the math says this works)
      // Least-squares expression linearized around the translation that brings the point (p1 or p2)
      // to its closest point on the circle
      phi(i * 2, 0) = 0.f;
      phi(i * 2, 1) = 0.f;
      phi(i * 2, 2) = w1 * v1.x / v1l;
      phi(i * 2, 3) = w1 * v1.y / v1l;
      k(i * 2) = w1 * (inter1.x * v1.x / v1l + inter1.y * v1.y / v1l);
      phi(i * 2 + 1, 0) = 0.f;
      phi(i * 2 + 1, 1) = 0.f;
      phi(i * 2 + 1, 2) = w2 * v2.x / v2l;
      phi(i * 2 + 1, 3) = w2 * v2.y / v2l;
      k(i * 2 + 1) = w2 * (inter2.x * v2.x / v2l + inter2.y * v2.y / v2l);

    } else if (matches[i].prior.type == Prior::Type::Segment) {

      const auto& fieldLine = matches[i].prior.segment;
      const float cosA = cos(fieldLine.alpha);
      const float sinA = sin(fieldLine.alpha);
      const float d = fieldLine.d;

      phi(i * 2, 0) = w1 * (cosA * p1.x + sinA * p1.y);
      phi(i * 2, 1) = w1 * (sinA * p1.x - cosA * p1.y); // -cosA is not a typo
      phi(i * 2, 2) = w1 * cosA;
      phi(i * 2, 3) = w1 * sinA;
      k(i * 2) = w1 * d;
      phi(i * 2 + 1, 0) = w2 * (cosA * p2.x + sinA * p2.y);
      phi(i * 2 + 1, 1) = w2 * (sinA * p2.x - cosA * p2.y); // -cosA is not a typo
      phi(i * 2 + 1, 2) = w2 * cosA;
      phi(i * 2 + 1, 3) = w2 * sinA;
      k(i * 2 + 1) = w2 * d;
    }
  }

  // Regularizer
  phi(2 * M, 0) = 0.f;
  phi(2 * M, 1) = 0.f;
  phi(2 * M, 2) = 1e-6f * regTransCorr;
  phi(2 * M, 3) = 0.f;
  k(2 * M) = 0.f;
  phi(2 * M + 1, 0) = 0.f;
  phi(2 * M + 1, 1) = 0.f;
  phi(2 * M + 1, 2) = 0.f;
  phi(2 * M + 1, 3) = 1e-6f * regTransCorr;
  k(2 * M + 1) = 0.f;

  // Optimize over rotation & translation, but only extract rotation
  STOP_TIME_ON_REQUEST("trackPose:solveBoth", {
    theta = phi.colPivHouseholderQr().solve(k);
    covar = 0.5f * (phi.transpose() * phi).inverse();
  });

  return true;
}

// Returns theta, the first row of a 2x2 matrix [c s; -s c] that best fits the matches
bool SelfLocator::fitRotation(const std::vector<Match>& matches,
                              Eigen::Vector2f& theta,
                              Eigen::Matrix2f& covar,
                              const Pose2D preTransform) const {

  using std::cos;
  using std::sin;

  const int M = matches.size();
  if (M == 0) {
    return false;
  }

  Eigen::Matrix<float, Eigen::Dynamic, 2> phi(M * 2, 2); // phi = phi(:, 0:1)
  Eigen::Matrix<float, Eigen::Dynamic, 1> k(M * 2, 1);

  const float preCos = cos(preTransform.rotation);
  const float preSin = sin(preTransform.rotation);
  const float preX = preTransform.translation.x;
  const float preY = preTransform.translation.y;

  for (int i = 0; i < M; ++i) {

    if (matches[i].prior.type == Prior::Type::Circle) {
      // Don't estimate rotation for matches to circles
      phi(i * 2, 0) = 0.f;
      phi(i * 2, 1) = 0.f;
      k(i * 2) = 0.f;
      phi(i * 2 + 1, 0) = 0.f;
      phi(i * 2 + 1, 1) = 0.f;
      k(i * 2 + 1) = 0.f;
      continue;
    }

    const auto& fieldLine = matches[i].prior.segment;
    const auto& observation = matches[i].observation;

    const float cosA = cos(fieldLine.alpha);
    const float sinA = sin(fieldLine.alpha);
    const float d = fieldLine.d;
    const Vector2<> p1 = Vector2<>(preCos * observation.p1.x - preSin * observation.p1.y + preX,
                                   preSin * observation.p1.x + preCos * observation.p1.y + preY);
    const Vector2<> p2 = Vector2<>(preCos * observation.p2.x - preSin * observation.p2.y + preX,
                                   preSin * observation.p2.x + preCos * observation.p2.y + preY);
    const float ol = (p1 - p2).abs();

    const float u1 =
      p1.abs() / ol; // different approx of perspective-induced uncertainty; distance matters less, length matters more
    const float w1 = 1.f / (u1 + 1.f);
    const float u2 = p2.abs() / ol;
    const float w2 = 1.f / (u2 + 1.f);

    phi(i * 2, 0) = w1 * (cosA * p1.x + sinA * p1.y);
    phi(i * 2, 1) = w1 * (sinA * p1.x - cosA * p1.y); // -cosA is not a typo
    k(i * 2) = w1 * d;
    phi(i * 2 + 1, 0) = w2 * (cosA * p2.x + sinA * p2.y);
    phi(i * 2 + 1, 1) = w2 * (sinA * p2.x - cosA * p2.y); // -cosA is not a typo
    k(i * 2 + 1) = w2 * d;
  }

  STOP_TIME_ON_REQUEST("trackPose:solveRot", {
    theta = phi.colPivHouseholderQr().solve(k);
    covar = 0.5f * (phi.transpose() * phi).inverse();
  });

  if (isnan(theta(0)) || isnan(theta(1))) {
    theta(0) = 1.f;
    theta(1) = 0.f;
    return false;
  }

  return true;
}

bool SelfLocator::fitTranslation(const std::vector<Match>& matches,
                                 Eigen::Vector2f& theta,
                                 Eigen::Matrix2f& covar,
                                 const Pose2D preTransform) const {

  using std::cos;
  using std::sin;

  const int M = matches.size();
  if (M == 0) {
    return false;
  }

  Eigen::Matrix<float, Eigen::Dynamic, 2> phi(M * 2 + 2, 2);
  Eigen::Matrix<float, Eigen::Dynamic, 1> k(M * 2 + 2, 1);

  const float preCos = cos(preTransform.rotation);
  const float preSin = sin(preTransform.rotation);
  const float preX = preTransform.translation.x;
  const float preY = preTransform.translation.y;

  for (int i = 0; i < M; ++i) {
    const auto& observation = matches[i].observation;

    const Vector2<> p1 = Vector2<>(preCos * observation.p1.x - preSin * observation.p1.y + preX,
                                   preSin * observation.p1.x + preCos * observation.p1.y + preY);
    const Vector2<> p2 = Vector2<>(preCos * observation.p2.x - preSin * observation.p2.y + preX,
                                   preSin * observation.p2.x + preCos * observation.p2.y + preY);
    const float ol = (p1 - p2).abs();

    const float u1 = p1.squareAbs() / ol; // only an approximation of perspective-induced uncertainty
    const float w1 = 1.f / (u1 + 1000.f); // PARAM distanceSoftening
    const float u2 = p2.squareAbs() / ol; // only an approximation of perspective-induced uncertainty
    const float w2 = 1.f / (u2 + 1000.f); // PARAM distanceSoftening

    if (matches[i].prior.type == Prior::Type::Circle) {

      const auto& circle = matches[i].prior.circle;
      const Vector2<>& c = circle.c;
      const float r = circle.r;

      // TODO Fit midpoint of observed segment too
      // const Vector2<> om = (p1 + p2)*0.5f;
      // const Vector2<> v = om - c; // circle center --> point
      // const float vl = v.abs();
      // const Vector2<> inter = v * (r/vl - 1.f);

      // const float u = om.squareAbs()/ol;
      // const float w = 1.f/(u + 1000.f);

      const Vector2<> v1 = p1 - c;
      const float v1l = v1.abs();
      const Vector2<> inter1 = v1 * (r / v1l - 1.f);

      const Vector2<> v2 = p2 - c;
      const float v2l = v2.abs();
      const Vector2<> inter2 = v2 * (r / v2l - 1.f);

      // Approximately fit translation to circle (don't ask me, the math says this works)
      // Least-squares expression linearized around the translation that brings the point (p1 or p2)
      // to its closest point on the circle
      phi(i * 2, 0) = w1 * v1.x / v1l;
      phi(i * 2, 1) = w1 * v1.y / v1l;
      k(i * 2) = w1 * (inter1.x * v1.x / v1l + inter1.y * v1.y / v1l);
      phi(i * 2 + 1, 0) = w2 * v2.x / v2l;
      phi(i * 2 + 1, 1) = w2 * v2.y / v2l;
      k(i * 2 + 1) = w2 * (inter2.x * v2.x / v2l + inter2.y * v2.y / v2l);

    } else if (matches[i].prior.type == Prior::Type::Segment) {

      const auto& fieldLine = matches[i].prior.segment;
      const float cosA = cos(fieldLine.alpha);
      const float sinA = sin(fieldLine.alpha);
      const float d = fieldLine.d;

      phi(i * 2, 0) = w1 * cosA;
      phi(i * 2, 1) = w1 * sinA;
      k(i * 2) = w1 * (d - cosA * p1.x - sinA * p1.y);
      phi(i * 2 + 1, 0) = w2 * cosA;
      phi(i * 2 + 1, 1) = w2 * sinA;
      k(i * 2 + 1) = w2 * (d - cosA * p2.x - sinA * p2.y);
    }
  }

  // Regularizer
  phi(2 * M, 0) = 1e-6f * regTransCorr;
  phi(2 * M, 1) = 0;
  k(2 * M) = 0.f;
  phi(2 * M + 1, 0) = 0;
  phi(2 * M + 1, 1) = 1e-6f * regTransCorr;
  k(2 * M + 1) = 0.f;

  STOP_TIME_ON_REQUEST("trackPose:solveTrans", {
    theta = phi.colPivHouseholderQr().solve(k);
    covar = 0.5f * (phi.transpose() * phi).inverse();
  });

  if (isnan(theta(0)) || isnan(theta(1))) {
    theta(0) = 0.f;
    theta(1) = 0.f;
    return false;
  }

  return true;
}

// Transforms lines into robot-centric field space, and appends them to priors
void SelfLocator::relativePriors(const Pose2D pose, std::vector<Prior>& priors) const {
  const Vector2<> p = pose.translation;
  const float r = pose.rotation;
  const float cosR = cos(r), sinR = sin(r);

  for (auto line : theFieldDimensions.getAllFieldLines()) {

    if (line.isPartOfCircle) {
      continue;
    }

    const float len = line.length();
    if (len < 200.f) {
      continue; // PARAM minLen; avoid penalty mark from LinesTable
    }

    const float rot = line.rotation() - r;
    float alpha = rot + 0.5f * (float)M_PI; // 90 deg left & normal to (p1->p2)
    while (alpha < 0) {
      alpha += (float)M_PI;
    }
    while (alpha >= M_PI) {
      alpha -= (float)M_PI;
    }

    // Segment start, translated so robot is at origin
    const Vector2<> rel = Vector2<>(line.from.x - p.x, line.from.y - p.y);

    // Segment start + end, in relative frame (translated + rotated)
    Vector2<> p1 = Vector2<>(cosR * rel.x + sinR * rel.y, -sinR * rel.x + cosR * rel.y);
    Vector2<> p2 = Vector2<>(p1.x + len * cos(rot), p1.y + len * sin(rot)); // turn 90 deg right for the actual line

    const float d = p1.x * cos(alpha) + p1.y * sin(alpha);

    priors.push_back(Prior(Segment(alpha, d, p1, p2)));
  }

  // Push center circle
  priors.push_back(
    Prior(Circle(Vector2<>(cosR * -p.x + sinR * -p.y, -sinR * -p.x + cosR * -p.y), theFieldDimensions.centerCircleRadius)));
}

// Clip prior lines within polyIn (assumed convex & CCW)
std::vector<SelfLocator::Prior> SelfLocator::clipPriors(const std::vector<Prior>& priors,
                                                        const std::vector<Vector2<>>& polyIn,
                                                        const Geometry::Circle sightRange) const {
  std::vector<Prior> clipped;

  // Close the loop: start and end with same point
  std::vector<Vector2<>> poly = polyIn;
  poly.push_back(poly[0]);

  const bool clipToSightRange = std::isfinite(sightRange.radius);

  for (const auto& prior : priors) {

    if (prior.type == Prior::Type::Circle) {
      // Circles: take entire circle if any part of it touches polygon; else reject entire circle
      const Vector2<> p = prior.circle.c;
      const float r = prior.circle.r;

      bool keep = true;

      // Exclude any circle whose center is not within sight range
      if (p.abs() >= sightRange.radius) {
        keep = false;
      }

      for (auto i = poly.begin(); i != poly.end() - 1; ++i) {
        const Vector2<> ps = *i;
        const Vector2<> pe = *(i + 1);

        const Vector2<> rel = p - ps;
        const Vector2<> n = (pe - ps).left().normalize();

        const float distanceOutside = -rel.dot(n); // "left" of (ps .. pe) is "inside", so we take negative to get "outside"
        if (distanceOutside > r) {
          keep = false;
        }
      }

      if (keep) {
        clipped.push_back(prior);
      }

    } else {
      // Line segments: clip line segment to polygon; reject if completely outside
      const auto& seg = prior.segment;

      Vector2<> p1 = seg.p1;
      Vector2<> p2 = seg.p2;

      // Clip p1, p2 to the half-space formed by each line in poly separately
      bool keep = true;
      for (auto i = poly.begin(); i != poly.end() - 1; ++i) {
        const Vector2<> ps = *i;
        const Vector2<> pe = *(i + 1);

        const Vector2<> rel1 = p1 - ps;
        const Vector2<> rel2 = p2 - ps;
        const Vector2<> n = (pe - ps).left();

        const bool p1Outside = (rel1.dot(n) < 0.f);
        const bool p2Outside = (rel2.dot(n) < 0.f);
        if (p1Outside && p2Outside) {
          keep = false;
          break;

        } else {
          if (p1Outside) {
            p1 = Vector2<>( // intersect(a->b, s->e)
              ((p1.x * p2.y - p1.y * p2.x) * (ps.x - pe.x) - (p1.x - p2.x) * (ps.x * pe.y - ps.y * pe.x)) /
                ((p1.x - p2.x) * (ps.y - pe.y) - (p1.y - p2.y) * (ps.x - pe.x)),
              ((p1.x * p2.y - p1.y * p2.x) * (ps.y - pe.y) - (p1.y - p2.y) * (ps.x * pe.y - ps.y * pe.x)) /
                ((p1.x - p2.x) * (ps.y - pe.y) - (p1.y - p2.y) * (ps.x - pe.x)));

          } else if (p2Outside) {
            p2 = Vector2<>( // intersect(a->b, s->e)
              ((p1.x * p2.y - p1.y * p2.x) * (ps.x - pe.x) - (p1.x - p2.x) * (ps.x * pe.y - ps.y * pe.x)) /
                ((p1.x - p2.x) * (ps.y - pe.y) - (p1.y - p2.y) * (ps.x - pe.x)),
              ((p1.x * p2.y - p1.y * p2.x) * (ps.y - pe.y) - (p1.y - p2.y) * (ps.x * pe.y - ps.y * pe.x)) /
                ((p1.x - p2.x) * (ps.y - pe.y) - (p1.y - p2.y) * (ps.x - pe.x)));
          }
        }
      }

      // Clip p1, p2 to sightRange
      if (clipToSightRange) {
        const bool isInsideCircle = Geometry::clipLineWithCircle(sightRange, p1, p2);
        if (!isInsideCircle) {
          keep = false;
        }
      }

      if (keep) {
        clipped.push_back({Segment{seg.alpha, seg.d, p1, p2}});
      }
    }
  }

  return clipped;
}

void SelfLocator::initWalkInPositions() {
  if (theGameInfo.competitionType == COMPETITION_TYPE_1VS1_CHALLENGE) {
    Pose2D leftPose = {-pi_2, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosLeftSideline};
    Pose2D rightPose = {pi_2, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosRightSideline};

    for (int i = 0; i < numberOfSamples; ++i) {
      if (i % 2) {
        samples->at(i).init(leftPose, defaultPoseDeviation);
      } else {
        samples->at(i).init(rightPose, defaultPoseDeviation);
      }
    }
  } else if (theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT) {
    for (int i = 0; i < numberOfSamples; ++i) {
      Pose2D initPose = initPositions[theRobotInfo.number - 1];
      samples->at(i).init(initPose, defaultPoseDeviation);
    }
  } else {
    for (int i = 0; i < numberOfSamples; ++i) {
      float initPoseX = theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber
                          ? theFieldDimensions.xPosOpponentPenaltyMark - 1000.f
                          : theFieldDimensions.xPosOwnGroundline;
      samples->at(i).init(Pose2D(0.f, initPoseX, 0.f), defaultPoseDeviation, 1.f);
    }
  }
}

bool SelfLocator::killFlips() {
  bool killed = false;
  for (int i = 0; i < samples->size(); ++i) {
    const Pose2D& sample = samples->at(i).getPose();

    const float mirrorSafetyMargin = 8.f; // this many times closer to the mirror before it is killed
    if (mirrorSafetyMargin * poseDistance(Pose2D(pi) + sample, theRobotPose) < poseDistance(sample, theRobotPose)) {
      samples->at(i).invalidate();
      killed = true;
      std::cout << "Invalidating particle (" << toDegrees(sample.rotation) << " deg, " << sample.translation.x << " mm, "
                << sample.translation.y << " mm)"
                << " while RobotPose is (" << toDegrees(theRobotPose.rotation) << " deg, " << theRobotPose.translation.x
                << " mm, " << theRobotPose.translation.y << " mm)" << std::endl;
    }
  }
  return killed;
}

void SelfLocator::resample() {
  const int N = samples->size();

  float totalValidity = 0.f;
  for (int i = 0; i < N; ++i) {
    totalValidity += samples->at(i).computeValidity();
  }

  if (totalValidity <= 0.f) {
    return;
  }

  if (drawWhichSample >= 255) {
    std::cout << "resample {" << std::endl;
  }

  bool somethingChanged = false;
  std::vector<UKFSample> newSamples;

  // Pick new samples
  float cumulativeWeight = 0.f; // weight[i] = validity[i] / totalValidity
  int nextNewSampleIdx = 0;
  for (int i = 0; i < N; ++i) {
    const auto& sample = samples->at(i);

    if (nextNewSampleIdx != i) {
      somethingChanged = true;
    }

    cumulativeWeight += sample.validity / totalValidity; // "upper limit" to how much samples[i] gets sampled

    while ((nextNewSampleIdx + 0.5f) / (float)N < cumulativeWeight) {
      newSamples.push_back(sample);
      ++nextNewSampleIdx;
      if (drawWhichSample >= 255) {
        std::cout << "    sample " << i << "; nextNewSampleIdx = " << nextNewSampleIdx
                  << ", cumulativeWeight = " << cumulativeWeight << std::endl;
      }
    }
  }

  if (drawWhichSample >= 255) {
    std::cout << "}" << (somethingChanged ? " changed!" : " ") << "(" << newSamples.size() << ")" << std::endl;
  }

  // Put new samples in
  if (somethingChanged) {
    for (int i = 0; i < N; ++i) {
      samples->at(i) = newSamples[i];
    }
  }
}

std::vector<SelfLocator::Cluster> SelfLocator::clusterSamples(SampleSet<UKFSample>& samples,
                                                              const Pose2D& referencePose) const {
  const int N = samples.size();
  std::vector<int> membership(N, -1); // membership[i] = which "cluster index" the i-th sample belongs to
  int nextCluster = 0;

  // Cluster things together. Not perfect: "elongated clusters" might get split into two, depending on ordering of samples
  for (int i = 0; i < N; ++i) {
    const auto& sample1 = samples.at(i);

    int cluster = membership[i];
    if (cluster == -1) {
      cluster = nextCluster;
      membership[i] = nextCluster;
      ++nextCluster;
    }

    for (int j = i + 1; j < N; ++j) {
      const auto& sample2 = samples.at(j);
      if (membership[j] != -1) {
        continue;
      }

      // Rotation is also a component.. albeit at a completely different scale (rads <-> mm)
      const float dist = poseDistance(sample1.getPose(), sample2.getPose());
      if (dist < clusterMaxDist) {
        membership[j] = cluster;
      }
    }
  }

  std::vector<Cluster> clusters(nextCluster);
  for (int i = 0; i < N; ++i) {
    clusters[membership[i]].samples.push_back(&(samples.at(i)));
  }

  // Fill up cluster information
  for (auto& cluster : clusters) {
    for (auto& sample : cluster.samples) {
      sample->computeValidity();
    }

    const float totalValidity = std::accumulate(
      cluster.samples.begin(), cluster.samples.end(), 0.f, [](float v, UKFSample* sample) { return v + sample->validity; });

    const float maxValidity =
      std::accumulate(cluster.samples.begin(), cluster.samples.end(), 0.f, [](float v, UKFSample* sample) {
        return std::max(v, sample->validity);
      });

    const Vector2<> meanPos = std::accumulate(cluster.samples.begin(),
                                              cluster.samples.end(),
                                              Vector2<>(),
                                              [](const Vector2<>& v, UKFSample* sample) {
                                                return v + Vector2<>(sample->mean.x, sample->mean.y) * sample->validity;
                                              }) /
                              totalValidity;

    const Vector2<> meanHdg = std::accumulate(
      cluster.samples.begin(), cluster.samples.end(), Vector2<>(), [](const Vector2<>& v, UKFSample* sample) {
        return v + Vector2<>(sample->validity, 0.f).rotate(sample->mean.z);
      });

    cluster.validity = maxValidity;
    cluster.mean = (meanHdg == Vector2<>(0.f, 0.f)) ? Pose2D(0.f, meanPos) : Pose2D(meanHdg.angle(), meanPos);
    cluster.distance = poseDistance(cluster.mean, referencePose);
  }

  return clusters;
}

void SelfLocator::computeModel(RobotPose& robotPose) const {
  if (drawWhichSample >= 255) {
    std::cout << "computeModel {" << std::endl;
  }

  std::vector<Cluster> clusters = clusterSamples(*samples, theRobotPose);

  std::vector<Cluster*> clusterPtrs;
  for (Cluster& cluster : clusters) {
    clusterPtrs.push_back(&cluster);
  }

  // Sort by ascending distance
  std::vector<Cluster*> clustersByDistance = clusterPtrs;
  std::sort(
    clustersByDistance.begin(), clustersByDistance.end(), [](Cluster* a, Cluster* b) { return a->distance < b->distance; });

  // Sort by descending cluster validity
  std::vector<Cluster*> clustersByValidity = clusterPtrs;
  std::sort(
    clustersByValidity.begin(), clustersByValidity.end(), [](Cluster* a, Cluster* b) { return a->validity > b->validity; });

  // By default, choose the cluster nearest to previous RobotPose
  Cluster* chosenCluster = clustersByDistance[0];

  // ... but take most valid cluster if it is 0.1 better than (or closer to 100% than) the nearest cluster
  if (clustersByValidity[0]->validity >= clustersByDistance[0]->validity + 0.1f ||
      clustersByValidity[0]->validity >= 0.5f * (clustersByDistance[0]->validity + 1.f)) {
    chosenCluster = clustersByValidity[0];
    if (clustersByValidity[0] != clustersByDistance[0]) {
      std::cout << "Picking more valid cluster (" << clustersByValidity[0]->validity << ", "
                << clustersByValidity[0]->distance << " mm) over closer cluster (" << clustersByDistance[0]->validity << ", "
                << clustersByDistance[0]->distance << " mm)" << std::endl;
    }
  }

  // Print stuff
  for (int i = 0; i < samples->size(); ++i) {
    const auto& sample = samples->at(i);

    const int clusterIdx =
      std::distance(clusters.begin(), std::find_if(clusters.begin(), clusters.end(), [&](const Cluster& cluster) {
                      return std::find(cluster.samples.begin(), cluster.samples.end(), &sample) != cluster.samples.end();
                    }));
    const float validity = sample.validity;

    if (drawWhichSample >= 255) {
      std::cout << "    [" << i << "] = (cluster = " << clusterIdx << ", validity = " << std::round(validity * 1e3f) / 1e3f
                << ", " << toString(sample.mean) << ")" << std::endl;
    }

    switch (i) {
    case 0:
      PLOT("module:SelfLocator:validity0", validity);
      break;
    case 1:
      PLOT("module:SelfLocator:validity1", validity);
      break;
    case 2:
      PLOT("module:SelfLocator:validity2", validity);
      break;
    case 3:
      PLOT("module:SelfLocator:validity3", validity);
      break;
    case 4:
      PLOT("module:SelfLocator:validity4", validity);
      break;
    case 5:
      PLOT("module:SelfLocator:validity5", validity);
      break;
    }
  }

  float validity = chosenCluster->validity;
  if (drawWhichSample >= 255) {
    std::cout << "    validity = " << validity << std::endl;
  }

  PLOT("module:SelfLocator:validity", validity);

  robotPose.validity = validity;
  if (validity > validityThreshold) {
    robotPose.timeWhenLastValid = theFrameInfo.time;
  }
  robotPose.timeSinceLastValid = theFrameInfo.time - robotPose.timeWhenLastValid;

  robotPose.translation = chosenCluster->mean.translation;
  robotPose.rotation = chosenCluster->mean.rotation;

  if (drawWhichSample >= 255) {
    std::cout << "    robotPose = " << robotPose.rotation << ", " << robotPose.translation.x << ", "
              << robotPose.translation.y << std::endl;
  }

  UKFSample* bestSample = *std::max_element(chosenCluster->samples.begin(),
                                            chosenCluster->samples.end(),
                                            [](UKFSample* a, UKFSample* b) { return a->validity < b->validity; });

  Matrix3x3f cov = bestSample->getCov();
  robotPose.deviation = sqrt(std::max(cov[0].x, cov[1].y));
  robotPose.covariance[0][0] = cov[0][0];
  robotPose.covariance[0][1] = cov[0][1];
  robotPose.covariance[0][2] = cov[0][2];
  robotPose.covariance[1][0] = cov[1][0];
  robotPose.covariance[1][1] = cov[1][1];
  robotPose.covariance[1][2] = cov[1][2];
  robotPose.covariance[2][0] = cov[2][0];
  robotPose.covariance[2][1] = cov[2][1];
  robotPose.covariance[2][2] = cov[2][2];

  if (drawWhichSample >= 255) {
    std::cout << "        deviation = " << robotPose.deviation << std::endl;
  }
  if (drawWhichSample >= 255) {
    std::cout << "        validity = " << robotPose.validity << std::endl;
  }
  if (drawWhichSample >= 255) {
    std::cout << "}" << std::endl;
  }
}

void SelfLocator::draw() {
  DECLARE_DEBUG_DRAWING("module:SelfLocator:samples", "drawingOnField"); // Draws all hypotheses/samples on the field
  COMPLEX_DRAWING(
    "module:SelfLocator:samples", for (int i = 0; i < numberOfSamples; ++i) { samples->at(i).draw(); });

  DECLARE_DEBUG_DRAWING("module:SelfLocator:simples",
                        "drawingOnField"); // Draws all hypotheses/samples on the field in a simple way
  COMPLEX_DRAWING(
    "module:SelfLocator:simples", for (int i = 0; i < numberOfSamples; ++i) { samples->at(i).draw(true); });

  DECLARE_DEBUG_DRAWING("module:SelfLocator:templates", "drawingOnField"); // Draws all available templates
  COMPLEX_DRAWING("module:SelfLocator:templates", sampleGenerator.draw(););
}

void SelfLocator::drawPose(const Pose2D& pose, ColorRGBA color = ColorRGBA(255, 0, 0)) const {
  const auto alpha = color.a;
  Vector2<> bodyPoints[4] = {Vector2<>(55, 90), Vector2<>(-55, 90), Vector2<>(-55, -90), Vector2<>(55, -90)};
  for (int i = 0; i < 4; i++) {
    bodyPoints[i] = pose * bodyPoints[i];
  }
  Vector2<> dirVec(200, 0);
  dirVec = pose * dirVec;
  LINE("module:SelfLocator:corrected",
       pose.translation.x,
       pose.translation.y,
       dirVec.x,
       dirVec.y,
       20,
       Drawings::ps_solid,
       ColorRGBA(255, 255, 255, alpha));
  POLYGON("module:SelfLocator:corrected",
          4,
          bodyPoints,
          20,
          Drawings::ps_solid,
          color,
          Drawings::bs_solid,
          ColorRGBA(255, 255, 255, alpha));
  CIRCLE("module:SelfLocator:corrected",
         pose.translation.x,
         pose.translation.y,
         42,
         0,
         Drawings::ps_solid,
         color,
         Drawings::bs_solid,
         color);

  DECLARE_DEBUG_DRAWING3D("module:SelfLocator:corrected", "field", {
    LINE3D("module:SelfLocator:corrected", pose.translation.x, pose.translation.y, 10, dirVec.x, dirVec.y, 10, 1, color);
    for (int i = 0; i < 4; ++i) {
      const Vector2<> p1 = bodyPoints[i];
      const Vector2<> p2 = bodyPoints[(i + 1) & 3];
      LINE3D("module:SelfLocator:corrected", p1.x, p1.y, 10, p2.x, p2.y, 10, 1, color);
    }
  });
}

// Field view: draw yellow lines for each match
// preTransform applies only to observations
void SelfLocator::drawMatches(const std::vector<Match>& matches, const Pose2D preTransform) const {
  COMPLEX_DRAWING("module:SelfLocator:matches", {
    for (auto m = matches.begin(); m != matches.end(); ++m) {
      const auto& i = m->prior;
      const auto& j = m->observation;

      Vector2<> pi, pj;
      if (i.type == Prior::Type::Circle) {
        const Circle& prior = i.circle;

        pj = Vector2<>((j.p1 + j.p2) * 0.5f).rotate(preTransform.rotation) + preTransform.translation;
        const Vector2<> vji = prior.c - pj;
        const float dist = vji.abs();
        pi = pj + vji * (dist - prior.r) / dist;

      } else if (i.type == Prior::Type::Segment) {
        const Segment& prior = i.segment;

        pj = Vector2<>((j.p1 + j.p2) * 0.5f).rotate(preTransform.rotation) + preTransform.translation;
        const float l = pj.x * cos(prior.alpha) + pj.y * sin(prior.alpha) - prior.d;
        pi = pj - Vector2<>(cos(prior.alpha), sin(prior.alpha)) * l;
      }
      const ColorRGBA color = m->inlier ? ColorClasses::yellow : ColorClasses::red;
      LINE("module:SelfLocator:matches", pj.x, pj.y, pi.x, pi.y, 5, Drawings::ps_solid, color);
      DRAWTEXT("module:SelfLocator:matches", (pj + pi).x / 2.f, (pj + pi).y / 2.f, 9, color, std::round(m->cost * 100.f));
    }
  });
}

void SelfLocator::drawCorrected(const std::vector<Segment>& observations, const Pose2D& correction, ColorRGBA color) const {
  COMPLEX_DRAWING("module:SelfLocator:corrected", {
    const float r = correction.rotation;

    for (auto i = observations.begin(); i != observations.end(); ++i) {
      const Vector2<> p1 =
        correction.translation + Vector2<>(cos(r) * i->p1.x - sin(r) * i->p1.y, sin(r) * i->p1.x + cos(r) * i->p1.y);
      const Vector2<> p2 =
        correction.translation + Vector2<>(cos(r) * i->p2.x - sin(r) * i->p2.y, sin(r) * i->p2.x + cos(r) * i->p2.y);
      const int thickness = 5;
      LINE("module:SelfLocator:corrected", p1.x, p1.y, p2.x, p2.y, thickness, Drawings::ps_solid, color);
    }
  });
}

void SelfLocator::drawPriors(const std::vector<Prior>& priors, ColorRGBA color) const {
  COMPLEX_DRAWING("module:SelfLocator:corrected", {
    for (auto i = priors.begin(); i != priors.end(); ++i) {
      if (i->type == Prior::Type::Segment) {
        const auto& segment = i->segment;
        const Vector2<>& p1 = segment.p1;
        const Vector2<>& p2 = segment.p2;
        const int thickness = 5;
        LINE("module:SelfLocator:corrected", p1.x, p1.y, p2.x, p2.y, thickness, Drawings::ps_solid, color);
      } else if (i->type == Prior::Type::Circle) {
        const auto& circle = i->circle;
        const Vector2<>& c = circle.c;
        const float r = circle.r;
        const int thickness = 5;
        CIRCLE("module:SelfLocator:corrected",
               c.x,
               c.y,
               r,
               thickness,
               Drawings::ps_solid,
               color,
               Drawings::bs_null,
               ColorClasses::none);
      }
    }
  });
}

MAKE_MODULE(SelfLocator, Modeling)

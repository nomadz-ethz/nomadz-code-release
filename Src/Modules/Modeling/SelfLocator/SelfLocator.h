/**
 * @file SelfLocator.h
 *
 * Declares a class that performs self-localization
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Geometry/Shapes.h"
#include "Tools/SelfLocator/FieldModel.h"
#include "Tools/SelfLocator/UKFSample.h"
#include "Tools/SelfLocator/TemplateGenerator.h"
#include "Tools/SelfLocator/SampleSet.h"
#include "Core/Module/Module.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/OwnSideModel.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/LineLocalization.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Perception/LinePercept.h"

#include <Eigen/Dense>

MODULE(SelfLocator)
REQUIRES(Odometer)
REQUIRES(OdometryData)
REQUIRES(OwnSideModel)
REQUIRES(OwnTeamInfo)
REQUIRES(GameInfo)
REQUIRES(RobotInfo)
REQUIRES(GoalPercept)
REQUIRES(LineAnalysis)
REQUIRES(FieldDimensions)
REQUIRES(FrameInfo)
REQUIRES(MotionInfo)
REQUIRES(CameraMatrix)
REQUIRES(BallModel)
REQUIRES(FallDownState)
REQUIRES(ArmContactModel)
REQUIRES(FieldBoundary)
REQUIRES(RobotPose)
REQUIRES(SideConfidence) //  <--- For GO 2013
REQUIRES(LineLocalization)
REQUIRES(CameraInfo)
REQUIRES(Image)
REQUIRES(LinePercept)
USES(CombinedWorldModel)
USES(BehaviorControlOutput)
USES(TeamMateData)
PROVIDES_WITH_OUTPUT(RobotPose)
PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(RobotPoseSamples)
LOADS_PARAMETER(int, numberOfSamples)                     /**< The number of samples used by the self-locator */
LOADS_PARAMETER(Pose2D, defaultPoseDeviation)             /**< Standard deviation used for creating new hypotheses */
LOADS_PARAMETER(Pose2D, filterProcessDeviation)           /**< The process noise for estimating the robot pose. */
LOADS_PARAMETER(Pose2D, odometryDeviation)                /**< The percentage inaccuracy of the odometry. */
LOADS_PARAMETER(Vector2<>, odometryRotationDeviation)     /**< A rotation deviation of each walked mm. */
LOADS_PARAMETER(float, goalAssociationMaxAngle)           /**< Only used in FieldModel */
LOADS_PARAMETER(float, goalAssociationMaxAngularDistance) /**< Only used in FieldModel */
LOADS_PARAMETER(
  float,
  lineAssociationCorridor) /**< Only used in FieldModel. The corridor used for relating seen lines with field lines. */
LOADS_PARAMETER(float, cornerAssociationDistance) /**< Only used in FieldModel */
LOADS_PARAMETER(float, centerCircleAssociationDistance)
LOADS_PARAMETER(Vector2<>, robotRotationDeviation) /**< Deviation of the rotation of the robot's torso */
LOADS_PARAMETER(Vector2<>,
                robotRotationDeviationInStand) /**< Deviation of the rotation of the robot's torso when he is standing. */
LOADS_PARAMETER(float, standardDeviationBallAngle)
LOADS_PARAMETER(float, standardDeviationBallDistance)
LOADS_PARAMETER(float, standardDeviationGoalpostSamplingDistance) /**< Only used in TemplateGenerator */
LOADS_PARAMETER(int, templateMaxKeepTime)                         /**< Only used in TemplateGenerator */
LOADS_PARAMETER(float, templateUnknownPostAssumptionMaxDistance)  /**< Only used in TemplateGenerator */
LOADS_PARAMETER(float, translationNoise)
LOADS_PARAMETER(float, rotationNoise)
LOADS_PARAMETER(float, movedDistWeight)
LOADS_PARAMETER(float, movedAngleWeight)
LOADS_PARAMETER(float, majorDirTransWeight)
LOADS_PARAMETER(float, minorDirTransWeight)
LOADS_PARAMETER(float, useRotationThreshold) /**< Below this distance from the field center;;; rotation influences the
                                                decision between pose and its mirror (mm). */
LOADS_PARAMETER(float, maxDistanceToFieldCenterForMirrorActions)
LOADS_PARAMETER(float, baseValidityWeighting)
LOADS_PARAMETER(float, sideConfidenceConfident)       /**< Value for side confidence when being definitely in own half */
LOADS_PARAMETER(float, sideConfidenceAlmostConfident) /**< Value for side confidence when being sure about own position but
                                                         not definitely in own half */
LOADS_PARAMETER(float, sideConfidenceConfused)        /**< Value for side confidence when bad things have happened */
LOADS_PARAMETER(Rangef, initPositionRange)

LOADS_PARAMETER(bool, resetRobotPose)      /**< robotPose can be manually reset in the representation robotPose */
LOADS_PARAMETER(float, centerDangerRadius) /**< when matching LineLocator hypotheses to samples, distance from field center
                                              within which rotation similarity becomes more important than translational
                                              distance */
LOADS_PARAMETER(bool, matchPriors)     /**< match priors to observations when calculating validity (in addition to matching
                                          observations to priors) */
LOADS_PARAMETER(float, priorClipRange) /**< when clipping priors before matching with observations, clip priors to
                                          robot-centered circle with this radius (-1 to disable) */
LOADS_PARAMETER(float, minInlierRatio) /**< particles with inlier ratio lower than this threshold are considered to be bad */
LOADS_PARAMETER(float, maxMatchCost)   /**< any matching above this cost is an outlier (in units of 10^-2) */
LOADS_PARAMETER(float, regTransCorr)   /**< regularizer for translation correction (in units of 10^-6) */
LOADS_PARAMETER(int, iterationsAll)    /**< number of match+transform iterations to run */
LOADS_PARAMETER(int, iterationsInliers) /**< number of match+inliers+transform iterations to run */
LOADS_PARAMETER(
  float, deviationPerDistance) /**< standard deviation of estimated LineLocator pose per distance of generating feature */
LOADS_PARAMETER(float, deviationR)        /**< standard deviation of rotation estimate (in radians) */
LOADS_PARAMETER(float, covarScale)        /**< scale the covariance returned by the line-fitting functions */
LOADS_PARAMETER(float, maxRotCorr)        /**< upper bound rotation correction */
LOADS_PARAMETER(float, maxTransCorr)      /**< upper bound translation correction */
LOADS_PARAMETER(float, clusterMaxDist)    /**< max dist from one point in cluster to closest point in same cluster */
LOADS_PARAMETER(bool, applyMotionUpdate)  /**< bool to enable motion update */
LOADS_PARAMETER(bool, applyLineLocator)   /**< bool to activate lineLocator procedrue */
LOADS_PARAMETER(bool, applyPoseTracking)  /**< bool to activate pose tracking procedure */
LOADS_PARAMETER(bool, applyFlipKilling)   /**< bool to activate killing poses that get close to being flipped */
LOADS_PARAMETER(bool, applyResampling)    /**< bool to activate resampling of particles */
LOADS_PARAMETER(float, validityThreshold) /**< robotPose considered invalid when validity below this (0 ... 1) */
LOADS_PARAMETER(int, drawWhichSample)     /**< draw pose tracker views for this sample */
LOADS_PARAMETER(int, poseValidTime)
LOADS_PARAMETER(int, poseLostTime)

END_MODULE

/**
 * @class SelfLocator
 *
 * A new module for self-localization
 */
class SelfLocator : public SelfLocatorBase {
private:
  class Segment {
  public:
    inline Segment() : alpha(0.f), d(0.f), p1(Vector2<>()), p2(Vector2<>()) {}
    inline Segment(const Segment& other) : alpha(other.alpha), d(other.d), p1(other.p1), p2(other.p2) {}
    inline Segment(float alpha, float d, Vector2<> p1, Vector2<> p2) : alpha(alpha), d(d), p1(p1), p2(p2) {}
    float alpha;
    float d;
    Vector2<> p1;
    Vector2<> p2;
  };

  class Circle {
  public:
    inline Circle() : c(Vector2<>()), r(0.f) {}
    inline Circle(const Circle& other) : c(other.c), r(other.r) {}
    inline Circle(Vector2<> c, float r) : c(c), r(r) {}
    Vector2<> c;
    float r;
  };

  class Prior {
  public:
    enum class Type { Segment, Circle } type;

    // unions give me headaches
    // union {
    Segment segment;
    Circle circle;
    // }

    inline Prior() : type() {}
    inline Prior(const Segment& s) : type(Type::Segment), segment(s) {}
    inline Prior(const Circle& c) : type(Type::Circle), circle(c) {}
  };

  class Match {
  public:
    Match(Prior prior, Segment observation, float cost, bool inlier = true)
        : prior(prior), observation(observation), cost(cost), inlier(inlier) {}
    Prior prior;
    Segment observation;
    float cost;
    bool inlier;
  };

  class Cluster {
  public:
    std::vector<UKFSample*> samples;
    float validity;
    Pose2D mean;
    float distance;
  };
  unsigned timePenaltyWasLeft;        /**<Time of when the penalty was left last>**/
  SampleSet<UKFSample>* samples;      /**< Container for all samples. */
  FieldModel fieldModel;              /**< Information about the robot's environment */
  TemplateGenerator sampleGenerator;  /**< Several parameters */
  float mirrorLikelihood;             /**< Value [0..1] indicating how many samples assume to be mirrored*/
  int lastPenalty;                    /**< Was the robot penalised in the last frame? */
  int lastGameState;                  /**< The game state in the last frame. */
  Pose2D lastRobotPose;               /**< The result of the last computation */
  unsigned timeOfLastFall;            /**< Timestamp to see if the robot has fallen down */
  unsigned lastTimeWithoutArmContact; /**< Timestamp for incorporating arm collisions into the model */
  unsigned lastTimeFarGoalSeen;       /**< Timestamp for checking goalie localization */
  bool sampleSetHasBeenResetted;      /**< Flag indicating that all samples have been replaced in the current frame */

  std::vector<int> badParticleCounter; /**< counts how many times a certain particle has been found to be bad */
  int noCorrectionCounter;             /**< counts how many times in a row no pose correction has been applied */
  int lostStateCounter;                /**< counts events that trigger lost state */
  std::vector<Pose2D> initPositions;
  std::vector<Segment>
    observations; /**< Pose tracking: Store field line observations from lower camer (for processing on upper) */

  /**
   * The method provides the robot pose (equal to the filtered robot pose).
   *
   * @param robotPose The robot pose representation that is updated by this module.
   */
  LineLocalization::Hypothesis lastHypothesis;
  void update(RobotPose& robotPose);

  /**
   * The method provides samples used by LinFindGoalPomdp in Behaviour section
   *
   * @param robotPoseSamples The robot pose samples representation that is updated by this module.
   */
  void update(RobotPoseSamples& robotPoseSamples);

  /**
   * The method provides the SideConfidence based on the uniformity of the distibution
   *
   * @param sideConfidence The side confidence
   */
  void update(SideConfidence& sideConfidence);

  /** Integrate odometry offset into hypotheses */
  void motionUpdate();

  void handleGameStateChanges(const Pose2D& propagatedRobotPose);

  UKFSample& getMostValidSample() const;

  UKFSample& getLeastValidSample() const;

  float poseDistance(const Pose2D&, const Pose2D&) const;
  bool isMirrorCloser(const Pose2D& samplePose, const Pose2D& robotPose) const;

  std::vector<Pose2D> runLineLocator(const Pose2D&);

  bool iterativeEstimator(const Pose2D& robotPose,
                          const std::vector<Prior>& priors,
                          const std::vector<Segment>& observations,
                          Pose2D& correction,
                          Matrix3x3f& covar,
                          bool debug = false) const;
  bool correctionWithinBounds(const Pose2D& original, const Pose2D& corrected) const;
  bool correctionWithinBounds(const Pose2D& correction) const;

  float priorMatchRatio(const std::vector<Prior>& priors,
                        const std::vector<Segment>& observations,
                        const Pose2D preTransform) const;

  void relativePriors(const Pose2D pose, std::vector<Prior>& priors) const;
  float matchObservations(const std::vector<Prior>& priors,
                          const std::vector<Segment>& observations,
                          std::vector<Match>& matches,
                          bool inliersOnly = true,
                          const Pose2D preTransform = Pose2D(),
                          bool output = true) const;
  void drawPov(const std::vector<Prior>& priors,
               const std::vector<Segment>& observations,
               const std::vector<Vector2<>>& fov,
               const Pose2D transform = {}) const;
  float matchRatio(const std::vector<Prior>& priors,
                   const std::vector<Segment>& observations,
                   const Pose2D pose,
                   const Pose2D preTransform = {}) const;
  std::vector<Prior> clipPriors(const std::vector<Prior>& priors,
                                const std::vector<Vector2<>>& polyIn,
                                const Geometry::Circle sightRange = {{}, INFINITY}) const;

  float copyInliers(const std::vector<Match>& matches, std::vector<Match>& inliers) const;
  float meanMatchCost(const std::vector<Match>& matches, bool inliersOnly = false) const;
  bool priorsAreOrientedSame(const std::vector<Match>& matches) const;

  bool estimateTransform(const std::vector<Match>& matches,
                         Pose2D& correction,
                         Matrix3x3f& covar,
                         const Pose2D preTransform = Pose2D(),
                         bool debug = false) const;
  bool fitBoth(const std::vector<Match>& matches,
               Eigen::Vector4f& theta,
               Eigen::Matrix4f& covar,
               const Pose2D preTransform = Pose2D()) const;
  bool fitRotation(const std::vector<Match>& matches,
                   Eigen::Vector2f& theta,
                   Eigen::Matrix2f& covar,
                   const Pose2D preTransform = Pose2D()) const;
  bool fitTranslation(const std::vector<Match>& matches,
                      Eigen::Vector2f& theta,
                      Eigen::Matrix2f& covar,
                      const Pose2D preTransform = Pose2D()) const;

  bool killFlips();

  void resample();

  std::vector<Cluster> clusterSamples(SampleSet<UKFSample>& samples, const Pose2D& referencePose) const;
  void computeModel(RobotPose& robotPose) const;

  void drawMatches(const std::vector<Match>& matches, const Pose2D preTransform = Pose2D()) const;
  void drawCorrected(const std::vector<Segment>& observations,
                     const Pose2D& correction,
                     ColorRGBA color = ColorRGBA(0, 0, 0)) const;
  void drawPriors(const std::vector<Prior>& priors, ColorRGBA color) const;

  /** Initialize hard coded walk-in positions */
  void initWalkInPositions();
  void computeInitPosition();
  /** draw debug information */
  void draw();

  void drawPose(const Pose2D& pose, ColorRGBA color) const;

public:
  /** Default constructor. */
  SelfLocator();

  /** Destructor. */
  ~SelfLocator();
};

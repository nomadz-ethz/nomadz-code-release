/**
 * @file Blackboard.h
 *
 * Declaration of a class representing the blackboard.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include <cstddef>
#include "Core/System/SystemCall.h"

// Declare prototypes of all representations here:

// Personal Data
class PersonalData;

// Infrastructure
class JointRequest;
class JointData;
class SensorData;
class KeyStates;
class LEDRequest;
class Image;
class CameraInfo;
class FrameInfo;
class CognitionFrameInfo;
class RobotInfo;
class OwnTeamInfo;
class OpponentTeamInfo;
class RawGameInfo;
class GameInfo;
class TeamMateData;
class MotionRobotHealth;
class RobotHealth;
class TeamDataSenderOutput;
class USRequest;
class Thumbnail;
class FsrSensorData;

// Configuration
class CameraSettings;
class FieldDimensions;
class RobotDimensions;
class JointCalibration;
class SensorCalibration;
class CameraCalibration;
class MassCalibration;
class HardnessSettings;
class DamageConfiguration;
class DamageConfigurationHead;
class HeadLimits;
class OdometryCorrectionTables;

// Perception
class CameraMatrix;
class RobotCameraMatrix;
class ImageCoordinateSystem;
class BallSpots;
class BallPercept;
class LineAnalysis;
class GoalPercept;
class GroundContactState;
class BodyContour;
class ColorReference;
class FieldBoundary;
class Features;
class ShapePercept;
class ImageAcquisition;
class LinePercept;
class PenaltyMarkPercept;
class PlayerPercept;
class RandomForests;
class RefereePercept;

// Modeling
class ArmContactModel;
class FallDownState;
class BallModel;
class BallModelAfterPreview;
class CombinedWorldModel;
class GroundTruthWorldState;
class GroundTruthBallModel;
class RobotPose;
class RobotPoseAfterPreview;
class RobotPoseSamples;
class FilteredRobotPose;
class FootContactModel;
class GroundTruthRobotPose;
class PlayerModel;
class GroundTruthPlayerModel;
class FieldCoverage;
class SideConfidence;
class Odometer;
class OwnSideModel;
class Whistle;
class AudioAcquisition;
class AudioData;
class LineLocalization;
class PassHelper;
class CoordinatedPassRepresentation;
class FixedOdometryRobotPose;

// BehaviorControl
class ActivationGraph;
class BehaviorControlOutput;
class BehaviorLEDRequest;
class Path;
class GoalSymbols;
class Plan;

// Sensing
class FilteredJointData;
class FilteredSensorData;
class InertiaSensorData;
class OrientationData;
class GroundTruthOrientationData;
class TorsoMatrix;
class RobotModel;
class RobotBalance;
class FsrData;
class FsrZmp;
class ArmContact;
class ZMPModel;
class FootSupport;

// MotionControl
class ArmMotionEngineOutput;
class ArmMotionRequest;
class ArmMovement;
class OdometryData;
class GroundTruthOdometryData;
class MotionRequest;
class SpeedRequest;
class HeadMotionRequest;
class HeadAngleRequest;
class HeadJointRequest;
class MotionSelection;
class SpecialActionsOutput;
class WalkingEngineOutput;
class WalkingEngineStandOutput;
class BikeEngineOutput;
class KickEngineOutput;
class MotionInfo;
struct KinematicRequest;
class JoystickControl;
class JoystickImageControl;
class ControllerParams;
class WalkingInfo;
class BodyTilt;
struct ActualCoM;
struct ActualCoMRCS;
struct ActualCoMFLIPM;
struct WalkingEngineParams;
class RefZMP;
class TargetCoM;
struct ReferenceModificator;
class FootSteps;
class Footpositions;
class ObservedFLIPMError;
class KinematicOutput;
struct SpeedInfo;
class PatternGenRequest;
struct WalkCalibration;
class ObservedError;
class FLIPMObserverParams;
class WalkGeneratorData;
class WalkKickGenerator;
class PlannedSteps;
class WalkLearner;
class Walk2014Modifier;

// friends
class Process;
class Cognition;
class Motion;
class Framework;
class CognitionLogger;

/**
 * @class Blackboard
 * The class represents the blackboard that contains all representation.
 * Note: The blackboard only contains references to the objects as attributes.
 * The actual representations are constructed on the heap, because many copies of
 * of the blackboard exist but only a single set of the representations shared
 * by all instances.
 */
class Blackboard {
protected:
  // Add all representations as constant references here:

  // Personal Data
  const PersonalData& thePersonalData;

  // Infrastructure
  const JointData& theJointData;
  const JointRequest& theJointRequest;
  const SensorData& theSensorData;
  const KeyStates& theKeyStates;
  const LEDRequest& theLEDRequest;
  const Image& theImage;
  const CameraInfo& theCameraInfo;
  const FrameInfo& theFrameInfo;
  const CognitionFrameInfo& theCognitionFrameInfo;
  const RobotInfo& theRobotInfo;
  const OwnTeamInfo& theOwnTeamInfo;
  const OpponentTeamInfo& theOpponentTeamInfo;
  const RawGameInfo& theRawGameInfo;
  const GameInfo& theGameInfo;
  const TeamMateData& theTeamMateData;
  const MotionRobotHealth& theMotionRobotHealth;
  const RobotHealth& theRobotHealth;
  const TeamDataSenderOutput& theTeamDataSenderOutput;
  const USRequest& theUSRequest;
  const Thumbnail& theThumbnail;
  const FsrSensorData& theFsrSensorData;

  // Configuration
  const CameraSettings& theCameraSettings;
  const FieldDimensions& theFieldDimensions;
  const RobotDimensions& theRobotDimensions;
  const JointCalibration& theJointCalibration;
  const SensorCalibration& theSensorCalibration;
  const CameraCalibration& theCameraCalibration;
  const MassCalibration& theMassCalibration;
  const HardnessSettings& theHardnessSettings;
  const DamageConfiguration& theDamageConfiguration;
  const DamageConfigurationHead& theDamageConfigurationHead;
  const HeadLimits& theHeadLimits;
  const OdometryCorrectionTables& theOdometryCorrectionTables;

  // Perception
  const CameraMatrix& theCameraMatrix;
  const RobotCameraMatrix& theRobotCameraMatrix;
  const ImageCoordinateSystem& theImageCoordinateSystem;
  const BallSpots& theBallSpots;
  const BallPercept& theBallPercept;
  const LineAnalysis& theLineAnalysis;
  const GoalPercept& theGoalPercept;
  const GroundContactState& theGroundContactState;
  const BodyContour& theBodyContour;
  const ColorReference& theColorReference;
  const FieldBoundary& theFieldBoundary;
  const Features& theFeatures;
  const ShapePercept& theShapePercept;
  const ImageAcquisition& theImageAcquisition;
  const LinePercept& theLinePercept;
  const PenaltyMarkPercept& thePenaltyMarkPercept;
  const PlayerPercept& thePlayerPercept;
  const RandomForests& theRandomForests;
  class RefereePercept& theRefereePercept;

  // Modeling
  const ArmContactModel& theArmContactModel;
  const FallDownState& theFallDownState;
  const BallModel& theBallModel;
  const BallModelAfterPreview& theBallModelAfterPreview;
  const CombinedWorldModel& theCombinedWorldModel;
  const GroundTruthWorldState& theGroundTruthWorldState;
  const GroundTruthBallModel& theGroundTruthBallModel;
  const RobotPose& theRobotPose;
  const RobotPoseAfterPreview& theRobotPoseAfterPreview;
  const RobotPoseSamples& theRobotPoseSamples;
  const FilteredRobotPose& theFilteredRobotPose;
  const FootContactModel& theFootContactModel;
  const GroundTruthRobotPose& theGroundTruthRobotPose;
  const PlayerModel& thePlayerModel;
  const GroundTruthPlayerModel& theGroundTruthPlayerModel;
  const FieldCoverage& theFieldCoverage;
  const SideConfidence& theSideConfidence;
  const Odometer& theOdometer;
  const OwnSideModel& theOwnSideModel;
  const Whistle& theWhistle;
  const AudioAcquisition& theAudioAcquisition;
  const AudioData& theAudioData;
  const LineLocalization& theLineLocalization;
  const PassHelper& thePassHelper;
  const CoordinatedPassRepresentation& theCoordinatedPassRepresentation;
  const FixedOdometryRobotPose& theFixedOdometryRobotPose;

  // BehaviorControl
  const ActivationGraph& theActivationGraph;
  const BehaviorControlOutput& theBehaviorControlOutput;
  const BehaviorLEDRequest& theBehaviorLEDRequest;
  const Path& thePath;
  const GoalSymbols& theGoalSymbols;
  const Plan& thePlan;

  // Sensing
  const FilteredJointData& theFilteredJointData;
  const FilteredSensorData& theFilteredSensorData;
  const InertiaSensorData& theInertiaSensorData;
  const OrientationData& theOrientationData;
  const GroundTruthOrientationData& theGroundTruthOrientationData;
  const TorsoMatrix& theTorsoMatrix;
  const RobotModel& theRobotModel;
  const RobotBalance& theRobotBalance;
  const FsrData& theFsrData;
  const FsrZmp& theFsrZmp;
  const ArmContact& theArmContact;
  const ZMPModel& theZMPModel;
  const FootSupport& theFootSupport;

  // MotionControl
  const ArmMotionEngineOutput& theArmMotionEngineOutput;
  const ArmMotionRequest& theArmMotionRequest;
  const ArmMovement& theArmMovement;
  const OdometryData& theOdometryData;
  const GroundTruthOdometryData& theGroundTruthOdometryData;
  const MotionRequest& theMotionRequest;
  const SpeedRequest& theSpeedRequest;
  const HeadAngleRequest& theHeadAngleRequest;
  const HeadMotionRequest& theHeadMotionRequest;
  const HeadJointRequest& theHeadJointRequest;
  const MotionSelection& theMotionSelection;
  const SpecialActionsOutput& theSpecialActionsOutput;
  const WalkingEngineOutput& theWalkingEngineOutput;
  const WalkingEngineStandOutput& theWalkingEngineStandOutput;
  const BikeEngineOutput& theBikeEngineOutput;
  const KickEngineOutput& theKickEngineOutput;
  const MotionInfo& theMotionInfo;
  const KinematicRequest& theKinematicRequest;
  const JoystickControl& theJoystickControl;
  const JoystickImageControl& theJoystickImageControl;
  const ControllerParams& theControllerParams;
  const WalkingInfo& theWalkingInfo;
  const BodyTilt& theBodyTilt;
  const ActualCoM& theActualCoM;
  const ActualCoMRCS& theActualCoMRCS;
  const ActualCoMFLIPM& theActualCoMFLIPM;
  const WalkingEngineParams& theWalkingEngineParams;
  const RefZMP& theRefZMP;
  const TargetCoM& theTargetCoM;
  const ReferenceModificator& theReferenceModificator;
  const FootSteps& theFootSteps;
  const Footpositions& theFootpositions;
  const ObservedFLIPMError& theObservedFLIPMError;
  const KinematicOutput& theKinematicOutput;
  const SpeedInfo& theSpeedInfo;
  const PatternGenRequest& thePatternGenRequest;
  const WalkCalibration& theWalkCalibration;
  const ObservedError& theObservedError;
  const FLIPMObserverParams& theFLIPMObserverParams;
  const WalkGeneratorData& theWalkGeneratorData;
  const WalkKickGenerator& theWalkKickGenerator;
  const PlannedSteps& thePlannedSteps;
  const WalkLearner& theWalkLearner;
  const Walk2014Modifier& theWalk2014Modifier;

  static PROCESS_WIDE_STORAGE(Blackboard) theInstance; /**< The only real instance in the current process. */

  /**
   * The method is a dummy that is called to prevent the compiler from certain
   * optimizations in a method generated in Module.h.
   * It is empty, but important, not defined inline.
   */
  static void distract();

protected:
  /**
   * Default constructor.
   */
  Blackboard();

public:
  /**
   * Virtual destructor.
   * Required for derivations of this class.
   */
  virtual ~Blackboard() {}

  /**
   * Assignment operator.
   * Note: copies will share all representations.
   * @param other The instance that is cloned.
   */
  void operator=(const Blackboard& other);

  /**
   * The operator allocates a memory block that is zeroed.
   * Therefore, all members of this class are initialized with 0.
   * @attention This operator is only called if this class is instantiated by
   * a separate call to new, i.e. it cannot be created as a part of another class.
   * @param size The size of the block in bytes.
   * @return A pointer to the block.
   */
  static void* operator new(std::size_t);

  /**
   * The operator frees a memory block.
   * @param p The address of the block to free.
   */
  static void operator delete(void* p);

  friend class Process;         /**< The class Process can set theInstance. */
  friend class Cognition;       /**< The class Cognition can read theInstance. */
  friend class Motion;          /**< The class Motion can read theInstance. */
  friend class Framework;       /**< The class Framework can set theInstance. */
  friend class CognitionLogger; /**< The cogniton logger needs to read theInstance */
};

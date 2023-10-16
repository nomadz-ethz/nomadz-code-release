/**
 * @file ROSRepresentationHelpers.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

// Explicitly instantiate ROS helpers for modules as this is extremely memory-heavy and slow.
// FIXME: Improve compile-time penalty of instantiation.

#include "ROSRepresentationHelpers.h"

#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/KeyStates.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointDataDeg.h"
#include "Representations/Infrastructure/Thumbnail.h"
#include "Representations/Infrastructure/AudioAcquisition.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/PersonalData.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/USRequest.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/GlobalFieldCoverage.h"
#include "Representations/Modeling/LineLocalization.h"
#include "Representations/Modeling/BallAfterKickPose.h"
#include "Representations/Modeling/PlayerModel.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/GroundTruthWorldState.h"
#include "Representations/Modeling/OwnSideModel.h"
#include "Representations/Perception/LineAnalysis.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/JPEGImage.h"
#include "Representations/Perception/RandomForestsData.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/PlayerPercept.h"
#include "Representations/Perception/JoystickImageControl.h"
#include "Representations/Perception/BallSpot.h"
#include "Representations/Perception/RandomForests.h"
#include "Representations/Perception/PenaltyMarkPercept.h"
#include "Representations/Perception/ColorReference.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/ShapePercept.h"
#include "Representations/Perception/ImageAcquisition.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/RefereePercept.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/FieldPosition.h"
#include "Representations/BehaviorControl/LocalSkill.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorConfig.h"
#include "Representations/BehaviorControl/GoalSymbols.h"
#include "Representations/BehaviorControl/Plan.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/OdometryCorrectionTable.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/SensorCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/CameraSettings.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/FsrZmp.h"
#include "Representations/Sensing/OrientationData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/FsrData.h"
#include "Representations/Sensing/ArmContact.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/RobotBalance.h"
#include "Representations/Sensing/FootContactModel.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/MotionControl/ArmMotionEngineOutput.h"
#include "Representations/MotionControl/WalkGeneratorData.h"
#include "Representations/MotionControl/PlannedSteps.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/SpecialActionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/JoystickControl.h"
#include "Representations/MotionControl/WalkRequest.h"
#include "Representations/MotionControl/WalkLearner.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/Walk2014Modifier.h"
#include "Representations/MotionControl/KickRequest.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/MotionRequest.h"

// Add new top-level representation below to instantiate helpers
// Standard
INSTANTIATE_ROS_HELPERS(ActivationGraph)
INSTANTIATE_ROS_HELPERS(ArmContact)
INSTANTIATE_ROS_HELPERS(ArmContactModel)
INSTANTIATE_ROS_HELPERS(ArmMotionEngineOutput)
INSTANTIATE_ROS_HELPERS(ArmMotionRequest)
INSTANTIATE_ROS_HELPERS(AudioAcquisition)
INSTANTIATE_ROS_HELPERS(AudioData)
INSTANTIATE_ROS_HELPERS(BallAfterKickPose)
INSTANTIATE_ROS_HELPERS(BallLocation)
INSTANTIATE_ROS_HELPERS(BallModel)
INSTANTIATE_ROS_HELPERS(BallPercept)
INSTANTIATE_ROS_HELPERS(BallSpots)
INSTANTIATE_ROS_HELPERS(BallState)
INSTANTIATE_ROS_HELPERS(BehaviorControlOutput)
INSTANTIATE_ROS_HELPERS(BehaviorLEDRequest)
INSTANTIATE_ROS_HELPERS(BehaviorStatus)
INSTANTIATE_ROS_HELPERS(FieldPosition)
INSTANTIATE_ROS_HELPERS(BodyContour)
INSTANTIATE_ROS_HELPERS(CameraCalibration)
INSTANTIATE_ROS_HELPERS(CameraMatrix)
INSTANTIATE_ROS_HELPERS(CameraSettings)
INSTANTIATE_ROS_HELPERS(ColorReference)
INSTANTIATE_ROS_HELPERS(CombinedWorldModel)
INSTANTIATE_ROS_HELPERS(DamageConfiguration)
INSTANTIATE_ROS_HELPERS(DamageConfigurationHead)
INSTANTIATE_ROS_HELPERS(FallDownState)
INSTANTIATE_ROS_HELPERS(FieldBoundary)
INSTANTIATE_ROS_HELPERS(FieldCoverage)
INSTANTIATE_ROS_HELPERS(FieldDimensions)
INSTANTIATE_ROS_HELPERS(FixedOdometryRobotPose)
INSTANTIATE_ROS_HELPERS(FootContactModel)
INSTANTIATE_ROS_HELPERS(FootSupport)
INSTANTIATE_ROS_HELPERS(FrameInfo)
INSTANTIATE_ROS_HELPERS(FsrData)
INSTANTIATE_ROS_HELPERS(FsrSensorData)
INSTANTIATE_ROS_HELPERS(FsrZmp)
INSTANTIATE_ROS_HELPERS(GaussianPositionDistribution)
INSTANTIATE_ROS_HELPERS(GlobalFieldCoverage)
INSTANTIATE_ROS_HELPERS(GoalPercept)
INSTANTIATE_ROS_HELPERS(GoalPost)
INSTANTIATE_ROS_HELPERS(GoalSymbols)
INSTANTIATE_ROS_HELPERS(GroundContactState)
INSTANTIATE_ROS_HELPERS(GroundTruthRobotPose)
INSTANTIATE_ROS_HELPERS(GroundTruthWorldState)
INSTANTIATE_ROS_HELPERS(HardnessData)
INSTANTIATE_ROS_HELPERS(HeadAngleRequest)
INSTANTIATE_ROS_HELPERS(HeadJointRequest)
INSTANTIATE_ROS_HELPERS(HeadLimits)
INSTANTIATE_ROS_HELPERS(HeadMotionRequest)
INSTANTIATE_ROS_HELPERS(ImageAcquisition)
INSTANTIATE_ROS_HELPERS(ImageCoordinateSystem)
INSTANTIATE_ROS_HELPERS(InertiaSensorData)
INSTANTIATE_ROS_HELPERS(JointData)
INSTANTIATE_ROS_HELPERS(JointRequest)
INSTANTIATE_ROS_HELPERS(JoystickControl)
INSTANTIATE_ROS_HELPERS(JoystickImageControl)
INSTANTIATE_ROS_HELPERS(KeyStates)
INSTANTIATE_ROS_HELPERS(KickEngineOutput)
INSTANTIATE_ROS_HELPERS(KickRequest)
INSTANTIATE_ROS_HELPERS(LEDRequest)
INSTANTIATE_ROS_HELPERS(LineAnalysis)
INSTANTIATE_ROS_HELPERS(LineLocalization)
INSTANTIATE_ROS_HELPERS(LinePercept)
INSTANTIATE_ROS_HELPERS(LocalSkill)
INSTANTIATE_ROS_HELPERS(RefereePercept)
INSTANTIATE_ROS_HELPERS(MassCalibration)
INSTANTIATE_ROS_HELPERS(MotionInfo)
INSTANTIATE_ROS_HELPERS(MotionRequest)
INSTANTIATE_ROS_HELPERS(MotionSelection)
INSTANTIATE_ROS_HELPERS(Odometer)
INSTANTIATE_ROS_HELPERS(OdometryCorrectionTable)
INSTANTIATE_ROS_HELPERS(OdometryCorrectionTable2D)
INSTANTIATE_ROS_HELPERS(OdometryCorrectionTables)
INSTANTIATE_ROS_HELPERS(OrientationData)
INSTANTIATE_ROS_HELPERS(OwnSideModel)
INSTANTIATE_ROS_HELPERS(PassTarget)
INSTANTIATE_ROS_HELPERS(PenaltyMarkPercept)
INSTANTIATE_ROS_HELPERS(PersonalData)
INSTANTIATE_ROS_HELPERS(PlayerModel)
INSTANTIATE_ROS_HELPERS(PlayerPercept)
INSTANTIATE_ROS_HELPERS(RandomForests)
INSTANTIATE_ROS_HELPERS(RandomForestsData)
INSTANTIATE_ROS_HELPERS(RandomForestsLoad)
INSTANTIATE_ROS_HELPERS(RobotBalance)
INSTANTIATE_ROS_HELPERS(RobotDimensions)
INSTANTIATE_ROS_HELPERS(RobotHealth)
INSTANTIATE_ROS_HELPERS(RobotModel)
INSTANTIATE_ROS_HELPERS(RobotPose)
INSTANTIATE_ROS_HELPERS(RobotPoseAfterPreview)
INSTANTIATE_ROS_HELPERS(RobotPoseCompressed)
INSTANTIATE_ROS_HELPERS(RobotPoseSamples)
INSTANTIATE_ROS_HELPERS(SensorCalibration)
INSTANTIATE_ROS_HELPERS(SensorData)
INSTANTIATE_ROS_HELPERS(ShapePercept)
INSTANTIATE_ROS_HELPERS(SideConfidence)
INSTANTIATE_ROS_HELPERS(SpecialActionRequest)
INSTANTIATE_ROS_HELPERS(SpecialActionsOutput)
INSTANTIATE_ROS_HELPERS(SpeedRequest)
INSTANTIATE_ROS_HELPERS(SystemSensorData)
INSTANTIATE_ROS_HELPERS(TeamDataSenderOutput)
INSTANTIATE_ROS_HELPERS(TeamHeadControlState)
INSTANTIATE_ROS_HELPERS(TeamMateData)
INSTANTIATE_ROS_HELPERS(TorsoMatrix)
INSTANTIATE_ROS_HELPERS(USRequest)
INSTANTIATE_ROS_HELPERS(Walk2014Modifier)
INSTANTIATE_ROS_HELPERS(WalkGeneratorData)
INSTANTIATE_ROS_HELPERS(PlannedSteps)
INSTANTIATE_ROS_HELPERS(WalkLearner)
INSTANTIATE_ROS_HELPERS(WalkRequest)
INSTANTIATE_ROS_HELPERS(WalkingEngineOutput)
INSTANTIATE_ROS_HELPERS(Whistle)

// Special
INSTANTIATE_ROS_HELPERS(CameraInfo)
INSTANTIATE_ROS_HELPERS(Image)
INSTANTIATE_ROS_HELPERS(RobotCameraMatrix)
INSTANTIATE_ROS_HELPERS(JointCalibration)
INSTANTIATE_ROS_HELPERS(HardnessSettings)
INSTANTIATE_ROS_HELPERS(CognitionFrameInfo)
INSTANTIATE_ROS_HELPERS(GroundTruthOrientationData)
INSTANTIATE_ROS_HELPERS(GroundTruthOdometryData)
INSTANTIATE_ROS_HELPERS(GroundTruthBallModel)
INSTANTIATE_ROS_HELPERS(GroundTruthPlayerModel)
INSTANTIATE_ROS_HELPERS(WalkingEngineStandOutput)
INSTANTIATE_ROS_HELPERS(Thumbnail)
INSTANTIATE_ROS_HELPERS(TeamInfo)
INSTANTIATE_ROS_HELPERS(OwnTeamInfo)
INSTANTIATE_ROS_HELPERS(OpponentTeamInfo)
INSTANTIATE_ROS_HELPERS(RobotInfo)
INSTANTIATE_ROS_HELPERS(OdometryData)
INSTANTIATE_ROS_HELPERS(FilteredSensorData)
INSTANTIATE_ROS_HELPERS(FilteredJointData)
INSTANTIATE_ROS_HELPERS(GameInfo)
INSTANTIATE_ROS_HELPERS(RawGameInfo)
INSTANTIATE_ROS_HELPERS(BallModelAfterPreview)
INSTANTIATE_ROS_HELPERS(MotionRobotHealth)
INSTANTIATE_ROS_HELPERS(KinematicRequest)

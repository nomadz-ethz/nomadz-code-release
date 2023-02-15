/**
 * @file MessageIDs.h
 *
 * Declaration of ids for debug messages.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Martin Lötzsch
 */

#pragma once

#include "Core/Enum.h"

/**
 * IDs for debug messages
 *
 * To distinguish debug messages, they all have an id.
 */
ENUM(MessageID,
     undefined,
     idProcessBegin,
     idProcessFinished,

     // data (ids should remain constant over code changes, so old log files will still work)
     idImage,
     idJPEGImage,
     idAudioData,
     idJointData,
     idSensorData,
     idKeyStates,
     idOdometryData,
     idFrameInfo,
     idFilteredJointData,
     idLineAnalysis,
     idGoalPercept,
     idBallPercept,
     idGroundTruthBallModel,
     idGroundTruthRobotPose,
     idObstacleSpots,
     idCameraMatrix,
     idCameraInfo,
     idImageCoordinateSystem,
     idMotionInfo,
     idRobotPose,
     idBallModel,
     idFilteredSensorData,
     idImageInfo,
     idOrientationData,
     idGameInfo,
     idRobotInfo,
     idOpponentTeamInfo,
     idSideConfidence,
     idPlayerModel,
     idGroundTruthPlayerModel,
     idGroundTruthOdometryData,
     idGroundTruthOrientationData,
     idColorReference,
     idOwnTeamInfo,
     idObstacleModel,
     idBehaviorControlOutput,
     idCombinedWorldModel,
     idFieldBoundary,
     idRobotHealth,
     idActivationGraph,
     idThumbnail,
     idRobotBalance,
     idStopwatch,
     idExpRobotPercept,
     idObstacleWheel,
     idBodyContour,
     idFeatures,
     idPenaltyMarkPercept,
     idPlayerPercept,
     idBallSpots,
     idShapePercept,
     idRandomForests,
     idLinePercept,
     idRobotPoseSamples,
     idPlan,
     idGroundTruthWorldState,
     idTeamMateData,
     // insert new data ids here
     idFootSupport,
     numOfDataMessageIDs, /**< everything below this does not belong into log files */

     // ids used in team communication
     idNTPHeader = numOfDataMessageIDs,
     idNTPIdentifier,
     idNTPRequest,
     idNTPResponse,
     idRobot,
     idTeamMateBallModel,
     idTeamMateObstacleModel,
     idTeamMateRobotPose,
     idTeamMateSideConfidence,
     idTeamMateBehaviorStatus,
     idMotionRequest,
     idTeamMateGoalPercept,
     idTeamMatePlayerModel,
     idTeamMateFreePartOfOpponentGoalModel,
     idTeamMateIsPenalized,
     idTeamMateHasGroundContact,
     idTeamMateIsUpright,
     idTeamMateCombinedWorldModel,
     idTeamHeadControl,
     idTeamMateTimeSinceLastGroundContact,
     idTeamCameraHeight,
     idTeamMateFieldCoverage,
     idTeamMatePlan,
     idObstacleClusters,
     idWhistle,
     // insert new team comm ids here

     idPersonalData,

     // infrastructure
     idText,
     idDebugRequest,
     idDebugResponse,
     idDebugDataResponse,
     idDebugDataChangeRequest,
     idStreamSpecification,
     idModuleTable,
     idModuleRequest,
     idQueueFillRequest,
     idLogResponse,
     idDrawingManager,
     idDrawingManager3D,
     idDebugImage,
     idDebugJPEGImage,
     idDebugDrawing,
     idDebugDrawing3D,
     idMotionNet,
     idJointRequest,
     idLEDRequest,
     idPlot,
     idConsole,
     idRobotname,
     idRobotDimensions,
     idJointCalibration,
     idUSRequest,
     idWalkingEngineKick,
     idDescriptorCommand,
     idKickEngineOutput);

/**
 * @file Options.h
 *
 * All option files that belong to the current behavior have to be included by this file.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 */

#include "Options/Soccer.h"

#include "Options/GameControl/HandleGamePhase.h"
#include "Options/GameControl/HandleGameState.h"
#include "Options/GameControl/PlayingState.h"
#include "Options/GameControl/ReadyState.h"
#include "Options/GameControl/SetState.h"

#include "Options/HeadControl/HeadControl.h"
#include "Options/HeadControl/LookForward.h"
#include "Options/HeadControl/LookAtBall.h"
#include "Options/HeadControl/LookAtTarget.h"
#include "Options/HeadControl/LookAtTargetWithTilt.h"
#include "Options/HeadControl/ScanLeftRight.h"
#include "Options/HeadControl/ScanRightLeft.h"
#include "Options/HeadControl/WalkScan.h"

#include "Options/Output/ArmMotionRequest/Arm.h"
#include "Options/Output/HeadMotionRequest/SetHeadPanTilt.h"
#include "Options/Output/HeadMotionRequest/SetHeadTargetOnGround.h"
#include "Options/Output/MotionRequest/InWalkKick.h"
#include "Options/Output/MotionRequest/PatternInwalkKick.h"
#include "Options/Output/MotionRequest/PatternAlignment.h"
#include "Options/Output/MotionRequest/PatternCircleAround.h"
#include "Options/Output/MotionRequest/SpecialAction.h"
#include "Options/Output/MotionRequest/Stand.h"
#include "Options/Output/MotionRequest/LookAtReferee.h"
#include "Options/Output/MotionRequest/StandWide.h"
#include "Options/Output/MotionRequest/StandHigh.h"
#include "Options/Output/MotionRequest/WalkAtSpeed.h"
#include "Options/Output/MotionRequest/WalkToTarget.h"
#include "Options/Output/PlaySound.h"

#include "Options/Functions/ShootOut.h"
#include "Options/Functions/Geometric.h"

#include "Options/Strategies/AllSearch.h"
#include "Options/Strategies/EngageBall.h"
#include "Options/Strategies/OffensivePlay.h"
#include "Options/Strategies/SupportPlay.h"
#include "Options/Strategies/DefendPosition.h"

#include "Options/Roles/HandleUncertainty.h"
#include "Options/Roles/ActiveRoleGenericBehavior.h"
#include "Options/Roles/Defender/Defender.h"
#include "Options/Roles/Defender/DefenderPosition.h"
#include "Options/Roles/Keeper/Keeper.h"
#include "Options/Roles/Keeper/KeeperSearch.h"
#include "Options/Roles/Striker/Striker.h"
#include "Options/Roles/PenaltyKeeper.h"
#include "Options/Roles/PenaltyStriker.h"
#include "Options/Roles/Supporter/Supporter.h"
#include "Options/Roles/TestRole.h"
#include "Options/Roles/VisualReferee/VisualRefereeChallenge.h"
#include "Options/Roles/VisualReferee/VisualRefereeReact.h"

#include "Options/Skills/PowerKick.h"
#include "Options/Skills/OmniKick.h"
#include "Options/Skills/GetUp.h"
#include "Options/Skills/DistancePass.h"
#include "Options/Skills/PassKick.h"
#include "Options/Skills/RefereeChallenge.h"

#include "Options/SkillSets/DribbleTo.h"
#include "Options/SkillSets/GetBehindBall.h"
#include "Options/SkillSets/OmniGetBehindBall.h"
#include "Options/SkillSets/BallAlignment.h"
#include "Options/SkillSets/KickTo.h"
#include "Options/SkillSets/InWalkKickTo.h"
// #include "Options/SkillSets/OmniKickTo.h"
#include "Options/SkillSets/PassTo.h"
#include "Options/SkillSets/ScoreGoal.h"
#include "Options/SkillSets/TravelTo.h"
#include "Options/SkillSets/WalkTo.h"
#include "Options/SkillSets/CircleAroundBall.h"

#include "Options/SkillSets/ScanTheField.h"

#include "Options/SkillSets/ScanRotate.h"
#include "Options/SkillSets/Relocate.h"
#include "Options/SkillSets/RelocateWhileSearchingBall.h"
#include "Options/SkillSets/RelocateWhileProtectBall.h"
#include "Options/SkillSets/SearchAndRelocate.h"

#include "Options/Tools/ButtonPressedAndReleased.h"
#include "Representations/MotionControl/PlannedSteps.h"

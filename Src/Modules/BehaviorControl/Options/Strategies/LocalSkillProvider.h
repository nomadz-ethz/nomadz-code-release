/**
 * @file LocalSkillProvider.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/LocalSkill.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Modeling/PlayerModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Core/Range.h"
#include "Core/Debugging/Debugging.h"
#include "Core/Module/Module.h"
#include "Core/Range.h"
#include "Core/Math/Vector.h"
#include "Core/Math/Constants.h"

MODULE(LocalSkillProvider)
REQUIRES(FrameInfo)
REQUIRES(GameInfo)
REQUIRES(RobotPose)
REQUIRES(BehaviorStatus)
REQUIRES(FieldDimensions)
REQUIRES(PersonalData)
REQUIRES(TeamMateData)
REQUIRES(RobotInfo)
REQUIRES(OwnTeamInfo)
REQUIRES(PlayerModel)
REQUIRES(BallModel)
PROVIDES_WITH_MODIFY(LocalSkill)
END_MODULE

class LocalSkillProvider : public LocalSkillProviderBase {

private:
  void update(LocalSkill& localSkill) override;
  Vector2<> calcReferenceTargetBallPosition();
  std::list<Range<float>> getFreeRanges();
  float calcNextTargetBallAngle(std::list<Range<float>> freeRanges);
  LocalSkill::Skill decideSkillType();
  float nearestOpponentInSight(float angle, float offset) const;
  std::vector<bool> isTeamMate(const std::vector<Vector2<>>& players) const;
  std::list<Range<float>> calcFreeRanges(const Vector2<>& ball, const Range<float>& goalRange);
  bool isCloseToGoal() const;
  bool isInDanger() const;
  Range<float> getBestFreeRange(const std::list<Range<float>>& freeRanges, const float robotToBallAngle);
  std::vector<int> getTeamMateIDs() const;
  LocalSkill::TargetProperty decideTargetProperty(const bool isCloseToGoal, LocalSkill::Skill skillType) const;
  void draw();

  Vector2<> referenceTargetBallPosition; // The Reference Global Target of the Ball. Usually the GoalPosition. But can be
                                         // adapted.
  LocalSkill::Skill skillType;           // The Skill Type to be executed
  float nextTargetBallAngle;             // The next Ball Position in robot frame
  float bestFreeRangesMin;               // temporarily
  float bestFreeRangesMax;               // temporarily
public:
  LocalSkillProvider();
};

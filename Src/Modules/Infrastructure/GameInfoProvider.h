/**
 * @file GameInfoProvider.h
 *
 * Provides the GameController packages, but makes sure the "set" or "play" game state is accurate.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"

MODULE(GameInfoProvider)
REQUIRES(RawGameInfo)
USES(Whistle)
USES(FrameInfo)
USES(OwnTeamInfo)
USES(BallModel)
USES(RobotPose)
PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GameInfo)
DEFINES_PARAMETER(float, ballTouchedThreshold, 200.f)
END_MODULE

class GameInfoProvider : public GameInfoProviderBase {
private:
  unsigned timeWhistleHeard; /**<Time of last hearing whistle>**/
  bool whistleHeard;         /**If whistle has been heard in this game**/
  void update(GameInfo& gameinfo);
  bool ballInPlay;

public:
  GameInfoProvider();
};

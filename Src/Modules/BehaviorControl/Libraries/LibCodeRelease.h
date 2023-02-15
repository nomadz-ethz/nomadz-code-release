/**
 * @file LibCodeRelease.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#include <math.h>

class LibCodeRelease : public LibraryBase {
public:
  /** Constructor for initializing all members*/
  LibCodeRelease();
  LibCodeRelease(internal::ref_init_tag t) : LibraryBase(t){};

  void preProcess() override;

  void postProcess() override;

  bool between(float value, float min, float max);

  int timeSinceBallWasSeen();

  float angleToOwnGoal;

  float angleToGoal;

  bool hasBallLock(bool wantsBall, float extraScore = 0);

  bool ballLockedByOthers();

  float nearestPlayerInSight(float angle, float maxoffset);

  float nearestOpponentInSight(float angle, float offset);

  std::vector<bool> isTeamMate(std::vector<Vector2<>> players, const std::vector<int>& teamMateIDs);

  bool isAnyTeamMateActive(std::initializer_list<int> playerNums);

  bool inGoalBox(const Vector2<> pos, float objectRadius);

  int numTeamMatesInGoalBox();

  bool nearCentre();

  std::list<Range<float>>
  calculateFreeRanges(const Vector2<>& ball, const Range<float>& goalRange, const std::vector<int>& teamMateIDs = {});

  Range<float> getBestFreeRange(const std::list<Range<float>>& freeRanges, const float robotToBallAngle);

  bool teamMateInFreeRange(const std::list<Range<float>>& freeRanges, const Vector2<>& gloBall, const int teamMateID);
};

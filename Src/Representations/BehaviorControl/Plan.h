/**
 * @file Plan.h
 *
 * Declares a class that stores a strategic plan.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <array>
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Core/Math/Vector2.h"
#include "Core/Streams/AutoStreamable.h"

STREAMABLE_DECLARE_LEGACY(BallLocation);
STREAMABLE_DECLARE_LEGACY(Plan);
STREAMABLE_DECLARE_LEGACY(PlanCompressed);

// In your dreams
// ENUM(BallLocation,
//   I, P1, P2, P3, P4, P5, G
// );

STREAMABLE_LEGACY(BallLocation, {
public:
  BallLocation(char v) : value(v) {}

  static const char I = 0; // can't use commas inside this stupid macro :(
  static const char P1 = 1;
  static const char P2 = 2;
  static const char P3 = 3;
  static const char P4 = 4;
  static const char P5 = 5;
  static const char G = 6;

  operator char() const { return value; }

  std::string name() const,

    (char)value,
});

STREAMABLE_LEGACY(Plan, {
public:
  int iter = 0;
  float timeCost = NAN;
  float rotnCost = NAN;
  float proxCost = NAN;
  float icptCost = NAN;
  float termCost = NAN;
  float anglCost = NAN;
  float edgeCost = NAN;
  std::vector<float> th; // K

  std::string cmd; // matlab command to plot plan

  Plan(float cost) : cost(cost) {}

  inline char K() const { return (char)b.size() - 1; }

  // Returns whether there is a plan to follow at all
  inline operator bool() const { return !b.empty(); }

  void doComplete() { step = K() + 1; }

  inline bool completed() const { return step > K(); }

  void clear();

  // Takes in player number (1 ... 5); returns whether this player has completed its part of the plan
  bool completedForMe(int pl) const;

  // Returns true if player "pl" is supposed to get the ball at any point
  bool isPlayerInvolved(int pl) const;

  // Takes in player number (1 ... 5); returns next step where player sees action (excluding the current step), or -1 if
  // failed
  char nextStepWithBall(int pl) const;

  void plotPaths(std::vector<Vector2<>> & ballPath, std::array<std::vector<Vector2<>>, 5> & playerPaths) const;

  bool canStep(const Vector2<>& ball, const std::vector<Vector2<>>& players) const;

  // Returns false if plan is complete
  bool doStep();

  // Draws things
  void draw(const RobotInfo&, const OwnTeamInfo&, const CombinedWorldModel&) const;

  std::string summarize() const;

  std::string str() const,

    (unsigned)(0)timeCreated, // NOT always accurate or the same (for the same plan) across different robots!
    (char)(-1)creator,        // robot number (1 .. 5)
    (float)(NAN)cost,
    (char)(-1)step,               // current step in plan; any step greater than K implies plan completed
    (std::vector<BallLocation>)b, // K + 1
    (std::vector<float>)x,        // K + 1
    (std::vector<float>)y,        // K + 1
    (std::vector<float>)t,        // K
});

STREAMABLE_LEGACY(PlanCompressed, {
public:
  PlanCompressed(const Plan& plan);

  operator Plan() const,

    (unsigned int)timeCreated,
    (char)creator,        // 1 .. 5
    (unsigned short)cost, // 0 .. 50 <> 0 .. 65535
    (char)step, (std::vector<char>)b,
    (std::vector<short>)x,          // -32768 .. 32767 <> -32768 .. 32767
    (std::vector<short>)y,          // -32768 .. 32767 <> -32768 .. 32767
    (std::vector<unsigned short>)t, // 0 .. 50 <> 0 .. 65535
});

/**
 * @file Plan.cpp
 *
 * Defines a class that stores a strategic plan.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <iomanip>
#include <ios>
#include <iostream>
#include <sstream>
#include "Plan.h"
#include "Core/Debugging/DebugDrawings3D.h"
#include "Core/Range.h"

std::string BallLocation::name() const {
  const std::vector<std::string> names = {"I", "1", "2", "3", "4", "5", "G"};
  return names[value];
}

void Plan::clear() {
  cost = timeCost = proxCost = icptCost = termCost = anglCost = edgeCost = NAN;
  b.clear();
  x.clear();
  y.clear();
  t.clear();
  step = -1;
  cmd = "";
}

bool Plan::completedForMe(int pl) const {
  if (!*this || completed()) {
    return true;
  }

  for (char k = step; k <= K(); ++k) {
    if (b[k] == (char)pl) {
      return false;
    }
  }

  return true;
}

bool Plan::isPlayerInvolved(int pl) const {
  for (char k = 0; k <= K(); ++k) {
    if (b[k] == (char)pl) {
      return true;
    }
  }

  return false;
}

char Plan::nextStepWithBall(int pl) const {
  for (char k = step + 1; k <= K(); ++k) {
    if (b[k] == (char)pl) {
      return k;
    }
  }

  return -1;
}

void Plan::plotPaths(std::vector<Vector2<>>& ballPath, std::array<std::vector<Vector2<>>, 5>& playerPaths) const {
  ballPath.clear();
  for (auto& playerPath : playerPaths) {
    playerPath.clear();
  }

  const char Np = 5;
  for (int k = 0; k <= K(); ++k) {
    for (char pl = 1; pl <= Np; ++pl) {
      if (b[k] == pl && k > 0) {
        playerPaths[pl - 1].push_back(Vector2<>(x[k], y[k]));
      }
    }

    ballPath.push_back(Vector2<>(x[k], y[k]));
  }
}

bool Plan::canStep(const Vector2<>& ball, const std::vector<Vector2<>>& players) const {
  return false;
}

bool Plan::doStep() {
  ++step;
  return !completed();
}

void Plan::draw(const RobotInfo& theRobotInfo,
                const OwnTeamInfo& theOwnTeamInfo,
                const CombinedWorldModel& theCombinedWorldModel) const {
  DECLARE_DEBUG_DRAWING3D("representation:Plan", "field");
  DECLARE_DEBUG_DRAWING("representation:Plan", "drawingOnField");

  if (!*this || this->completedForMe(theRobotInfo.number)) {
    return;
  }

  COMPLEX_DRAWING3D("representation:Plan", {
    // Player path
    for (unsigned char pl = 1; pl <= 5; ++pl) {
      std::vector<Vector2<>> playerPath;
      std::vector<int> penWidths;

      // Planned path
      for (char k = 0; k <= K(); ++k) {
        if (b[k] == pl) {
          playerPath.push_back(Vector2<>(x[k], y[k]));
          penWidths.push_back(step == k ? 15 : 6);
        }
      }

      for (char i = 0; i < (int)playerPath.size() - 1; ++i) {
        const Vector3<> from(playerPath[i].x, playerPath[i].y, 250.f);
        const Vector3<> to(playerPath[i + 1].x, playerPath[i + 1].y, 250.f);
        const ColorRGBA color =
          (theOwnTeamInfo.teamColor == TEAM_BLUE) ? ColorRGBA(0, 0, 255, 255) : ColorRGBA(255, 0, 0, 255);
        CYLINDERARROW3D("representation:Plan", from, to, penWidths[i], 100, 30, color);
      }

      // Real player path
      if (b[step] == pl) {
        // Player -> Ball -> Next step
        // Player -> Ball
        {
          const Vector3<> from(
            theCombinedWorldModel.ownPoses[pl].translation.x, theCombinedWorldModel.ownPoses[pl].translation.y, 250.f);
          const Vector3<> to(theCombinedWorldModel.ballState.position.x, theCombinedWorldModel.ballState.position.y, 250.f);
          const float penWidth = 15.f;
          const ColorRGBA color =
            (theOwnTeamInfo.teamColor == TEAM_BLUE) ? ColorRGBA(0, 0, 255, 255) : ColorRGBA(255, 0, 0, 255);
          CYLINDERARROW3D("representation:Plan", from, to, penWidth, 100, 30, color);
        }
        // Ball -> Next step
        {
          const Vector3<> from(
            theCombinedWorldModel.ballState.position.x, theCombinedWorldModel.ballState.position.y, 250.f);
          const Vector3<> to(x[step + 1], y[step + 1], 250.f);
          const float penWidth = 15.f;
          const ColorRGBA color =
            (theOwnTeamInfo.teamColor == TEAM_BLUE) ? ColorRGBA(0, 0, 255, 255) : ColorRGBA(255, 0, 0, 255);
          CYLINDERARROW3D("representation:Plan", from, to, penWidth, 100, 30, color);
        }
      } else {
        // Player -> Next step
        for (char k = step + 1; k <= K(); ++k) {
          if (b[k] == pl) {
            const Vector3<> from(
              theCombinedWorldModel.ownPoses[pl].translation.x, theCombinedWorldModel.ownPoses[pl].translation.y, 250.f);
            const Vector3<> to(x[k], y[k], 250.f);
            const float penWidth = 6.f;
            const ColorRGBA color =
              (theOwnTeamInfo.teamColor == TEAM_BLUE) ? ColorRGBA(0, 0, 255, 255) : ColorRGBA(255, 0, 0, 255);
            CYLINDERARROW3D("representation:Plan", from, to, penWidth, 100, 30, color);
            break;
          }
        }
      }
    }

    // Ball path
    std::vector<Vector2<>> ballPath;
    std::vector<int> penWidths;
    for (char k = 0; k <= K(); ++k) {
      const Vector2<> newPoint(x[k], y[k]);
      if (ballPath.empty() || ballPath.back() != newPoint) {
        ballPath.push_back(newPoint);
        penWidths.push_back(step == k ? 15 : 6);
      }
    }

    for (char i = 0; i < (int)ballPath.size() - 1; ++i) {
      const Vector3<> from(ballPath[i].x, ballPath[i].y, 50.f);
      const Vector3<> to(ballPath[i + 1].x, ballPath[i + 1].y, 50.f);
      const ColorRGBA color = (theOwnTeamInfo.teamColor == TEAM_BLUE) ? ColorRGBA(0, 0, 64, 255) : ColorRGBA(64, 0, 0, 255);
      CYLINDERARROW3D("representation:Plan", from, to, penWidths[i], 100, 30, color);
    }

    // Real ball path
    {
      const Vector3<> from(theCombinedWorldModel.ballState.position.x, theCombinedWorldModel.ballState.position.y, 50.f);
      const Vector3<> to(x[step + 1], y[step + 1], 50.f);
      const float penWidth = 15.f;
      const ColorRGBA color = (theOwnTeamInfo.teamColor == TEAM_BLUE) ? ColorRGBA(0, 0, 64, 255) : ColorRGBA(64, 0, 0, 255);
      CYLINDERARROW3D("representation:Plan", from, to, penWidth, 100, 30, color);
    }
  });

  COMPLEX_DRAWING("representation:Plan", {
    // Player path
    for (unsigned char pl = 1; pl <= 5; ++pl) {
      std::vector<Vector2<>> playerPath;
      std::vector<int> penWidths;
      std::vector<Drawings::PenStyle> penStyles;
      for (char k = 0; k <= K(); ++k) {
        if (step == k) {
          playerPath.push_back(theCombinedWorldModel.ownPoses[pl].translation);
          penWidths.push_back((k == step) ? 30 : 15);
          penStyles.push_back((k <= step) ? Drawings::ps_solid : Drawings::ps_dash);
        }
        if (b[k] == pl) {
          playerPath.push_back(Vector2<>(x[k], y[k]));
          penWidths.push_back((k == step) ? 30 : 15);
          penStyles.push_back((k <= step) ? Drawings::ps_solid : Drawings::ps_dash);
        }
      }

      for (char i = 0; i < (int)playerPath.size() - 1; ++i) {
        ARROW("representation:Plan",
              playerPath[i].x,
              playerPath[i].y,
              playerPath[i + 1].x,
              playerPath[i + 1].y,
              penWidths[i],
              penStyles[i],
              ColorRGBA(0, 0, 255, 255));
      }
    }

    // Ball path
    std::vector<Vector2<>> ballPath;
    std::vector<int> penWidths;
    std::vector<Drawings::PenStyle> penStyles;
    for (char k = 0; k <= K(); ++k) {
      if (step == k || b[k - 1] == BallLocation::I) {
        ballPath.push_back(theCombinedWorldModel.ballState.position);
        // } else {
      }
      ballPath.push_back(Vector2<>(x[k], y[k]));
      penWidths.push_back((k == step) ? 30 : 15);
      penStyles.push_back((k <= step) ? Drawings::ps_solid : Drawings::ps_dash);
    }

    for (char i = 0; i < (int)ballPath.size() - 1; ++i) {
      ARROW("representation:Plan",
            ballPath[i].x,
            ballPath[i].y,
            ballPath[i + 1].x,
            ballPath[i + 1].y,
            penWidths[i],
            penStyles[i],
            ColorRGBA(0, 0, 0, 255));
    }
  });
}

std::string Plan::summarize() const {
  std::ostringstream str;
  const char endl = '\n';

  str << "Summary" << endl;
  str << "( ";
  for (BallLocation bl : b) {
    str << bl.name() << " ";
  }
  str << "):" << endl;
  str << " Cost: " << cost << endl;
  str << "  x: [ ";
  for (int k = 0; k <= K(); ++k) {
    str << x[k] << " ";
  }
  str << "]" << endl;
  str << "  y: [ ";
  for (int k = 0; k <= K(); ++k) {
    str << y[k] << " ";
  }
  str << "]" << endl;
  str << "  time: " << timeCost << " [ ";
  for (auto n : t) {
    str << n << " ";
  }
  str << "]" << endl;
  str << "  rotn: " << rotnCost << " [ ";
  for (auto n : th) {
    str << toDegrees(std::acos(1 - 2.0 / M_PI * n)) << "(" << n << ") ";
  }
  str << "]" << endl;
  str << "  hdgs: [ ";
  for (int k = 1; k <= K(); ++k) {
    str << toDegrees(std::atan2(y[k] - y[k - 1], x[k] - x[k - 1])) << " ";
  }
  str << "]" << endl;
  str << "  dsts: [ ";
  for (int k = 1; k <= K(); ++k) {
    str << std::sqrt((x[k] - x[k - 1]) * (x[k] - x[k - 1]) + (y[k] - y[k - 1]) * (y[k] - y[k - 1])) << " ";
  }
  str << "]" << endl;
  str << std::setprecision(4) << std::fixed;
  str << "  edge: " << edgeCost << "\tangl: " << anglCost << endl;
  str << "  prox: " << proxCost << "\ticpt: " << icptCost << endl;
  str << "  term: " << termCost << endl;
  str << std::setprecision(6);
  str.unsetf(std::ios_base::floatfield);

  return str.str();
}

std::string Plan::str() const {
  std::ostringstream str;
  const char endl = '\n';

  str << "cost: " << cost << endl;
  str << "b: ";
  for (int k = 0; k <= K(); ++k) {
    str << b[k].name() << " ";
  }
  str << endl;
  str << "x: ";
  for (int k = 0; k <= K(); ++k) {
    str << x[k] << " ";
  }
  str << endl;
  str << "y: ";
  for (int k = 0; k <= K(); ++k) {
    str << y[k] << " ";
  }
  str << endl;
  str << "t: ";
  for (int k = 0; k < K(); ++k) {
    str << t[k] << " ";
  }
  str << endl;
  // str << cmd << endl;

  return str.str();
}

PlanCompressed::PlanCompressed(const Plan& plan)
    : timeCreated(plan.timeCreated), creator(plan.creator),
      cost(Range<unsigned short>(0, 65535).limit((unsigned short)std::round(plan.cost * 100.f))), step(plan.step) {

  for (const auto& bl : plan.b) {
    b.push_back(bl);
  }

  for (const auto& xi : plan.x) {
    x.push_back((unsigned short)(xi));
  }

  for (const auto& yi : plan.y) {
    y.push_back((unsigned short)(yi));
  }

  for (const auto& ti : plan.t) {
    t.push_back(Range<unsigned short>(0, 65535).limit((unsigned short)std::round(ti * 100.f)));
  }
}

PlanCompressed::operator Plan() const {
  Plan plan;
  plan.timeCreated = timeCreated;
  plan.creator = creator;
  plan.cost = cost / 100.f;
  plan.step = step;

  for (const auto& bl : b) {
    plan.b.push_back(bl);
  }

  for (const auto& xi : x) {
    plan.x.push_back(float(xi));
  }

  for (const auto& yi : y) {
    plan.y.push_back(float(yi));
  }

  for (const auto& ti : t) {
    plan.t.push_back(ti / 100.f);
  }

  return plan;
}

/**
 * @file CombinedWorldModel.cpp
 *
 * Implementation of a debug drawing of the combined world model
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Katharina Gillmann
 */

#include "CombinedWorldModel.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Debugging/DebugDrawings3D.h"
#include "Core/Math/Covariance.h"
#include "Core/Math/Vector3.h"

GaussianPositionDistribution::GaussianPositionDistribution(const Vector2<>& robotPosition, const Matrix2x2<>& covariance)
    : GaussianPositionDistribution() {
  this->robotPosition = robotPosition;
  this->covariance = covariance;
}

void CombinedWorldModel::draw() const {
  DECLARE_DEBUG_DRAWING3D(
    "representation:CombinedWorldModel",
    "field",
    // draw opponent robots
    for (std::vector<GaussianPositionDistribution>::const_iterator i = positionsOpponentTeam.begin();
         i != positionsOpponentTeam.end();
         ++i) {
      float xExpansion, yExpansion, rotation;
      Covariance::errorEllipse(i->covariance, xExpansion, yExpansion, rotation);
      CYLINDER3D("representation:CombinedWorldModel",
                 i->robotPosition.x,
                 i->robotPosition.y,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.0f,
                 35.0f,
                 20.0f,
                 ColorRGBA(0, 0, 255, 100));
    }
    // draw own team
    for (std::vector<Pose2D>::const_iterator i = ownPoses.begin(); i != ownPoses.end(); ++i) {
      const int idx = std::distance(ownPoses.begin(), i);
      if (!canPlay[idx])
        continue;
      CYLINDER3D("representation:CombinedWorldModel",
                 i->translation.x,
                 i->translation.y,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.0f,
                 35.0f,
                 60.0f,
                 ColorRGBA(0, 0, 0, 100));
    }

    // draw global ball
    const Vector3<> ballPos3d = Vector3<>(ballStateOthers.position.x, ballStateOthers.position.y, 0.0f);
    const Vector3<> ballSpeed3d = Vector3<>(ballStateOthers.velocity.x, ballStateOthers.velocity.y, 0.0f);
    SPHERE3D("representation:CombinedWorldModel", ballPos3d.x, ballPos3d.y, 35.f, 35.f, ColorRGBA(128, 64, 0));
    if (ballSpeed3d.squareAbs() > 0.9f) {
      CYLINDERARROW3D(
        "representation:CombinedWorldModel", ballPos3d, ballPos3d + ballSpeed3d, 5.f, 35.f, 35.f, ColorRGBA(128, 64, 0));
    });

  DECLARE_DEBUG_DRAWING("representation:CombinedWorldModel", "drawingOnField");

  COMPLEX_DRAWING("representation:CombinedWorldModel", {
    for (std::vector<Pose2D>::const_iterator i = ownPoses.begin(); i != ownPoses.end(); ++i) {
      const int idx = std::distance(ownPoses.begin(), i);
      if (!canTrust[idx])
        continue;

      const ColorRGBA color = canPlay[idx] ? ColorClasses::blue : ColorRGBA(128, 128, 128, 0);
      CIRCLE("representation:CombinedWorldModel",
             i->translation.x,
             i->translation.y,
             500,
             20,
             Drawings::ps_solid,
             color,
             Drawings::bs_null,
             ColorRGBA());

      Vector2<> dir(1.f, 0.f);
      dir.rotate(i->rotation);
      dir *= 250.f;
    }

    for (std::vector<GaussianPositionDistribution>::const_iterator i = positionsOpponentTeam.begin();
         i != positionsOpponentTeam.end();
         ++i) {
      CIRCLE("representation:CombinedWorldModel",
             i->robotPosition.x,
             i->robotPosition.y,
             300,
             20,
             Drawings::ps_solid,
             ColorRGBA(ColorClasses::yellow),
             Drawings::bs_null,
             ColorRGBA());
    }
    if (!ballIsValid)
      DRAWTEXT("representation:CombinedWorldModel", 1000, 1000, 10, ColorRGBA(ColorClasses::black), "ball invalid");
  });

  COMPLEX_DRAWING("representation:CombinedWorldModel", {
    CIRCLE("representation:CombinedWorldModel",
           ballStateOthers.position.x,
           ballStateOthers.position.y,
           30,
           20,
           Drawings::ps_solid,
           ColorRGBA(ColorClasses::red),
           Drawings::bs_null,
           ColorRGBA());
    ARROW("representation:CombinedWorldModel",
          ballStateOthers.position.x,
          ballStateOthers.position.y,
          ballStateOthers.position.x + ballStateOthers.velocity.x,
          ballStateOthers.position.y + ballStateOthers.velocity.y,
          5,
          1,
          ColorRGBA(ColorClasses::red));
  });
}

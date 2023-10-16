/**
 * @file SideConfidence.h
 *
 * Declaration of class SideConfidence.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are Michel Bartsch, Thomas Münder
 * and <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Enum.h"
#include "Core/Debugging/DebugDrawings3D.h"
#include "Core/Settings.h"

/**
 * @class SideConfidence
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/side_confidence.hpp"
#endif
STREAMABLE_DECLARE(SideConfidence)

STREAMABLE_ROS(SideConfidence, {
public:
  ENUM(ConfidenceState,
       CONFIDENT,
       ALMOST_CONFIDENT,
       UNSURE,
       CONFUSED); /**< Discrete states of confidence, mapped by provider */

  /** Draw representation. */
  void draw() const {
    DECLARE_DEBUG_DRAWING3D("representation:SideConfidence", "robot", {
      static const ColorRGBA colors[numOfConfidenceStates] = {
        ColorRGBA(0, 255, 0), ColorRGBA(0, 128, 0), ColorRGBA(255, 255, 0), ColorRGBA(255, 0, 0)};
      int pNumber = Global::getSettings().playerNumber;
      float centerDigit = (pNumber > 1) ? 50.f : 0;
      ROTATE3D("representation:SideConfidence", 0, 0, pi_2);
      DRAWDIGIT3D(
        "representation:SideConfidence", pNumber, Vector3<>(centerDigit, 0.f, 500.f), 80, 5, colors[confidenceState]);
    });

    DECLARE_DEBUG_DRAWING("representation:SideConfidence", "drawingOnField", {
      DRAWTEXT("representation:SideConfidence", -5000, -3600, 140, ColorClasses::red, "Sideconfidence: " << sideConfidence);
    });
  }
  ,
    FIELD_WRAPPER(
      float,
      1,
      nomadz_msgs::msg::SideConfidence::side_confidence,
      sideConfidence), /**< Am I mirrored because of two yellow goals (0 = no idea, 1 = absolute sure I am right). */
    FIELD_WRAPPER(bool,
                  false,
                  nomadz_msgs::msg::SideConfidence::mirror,
                  mirror), /**< Indicates whether ball model of others is mirrored to own ball model. */
    FIELD_WRAPPER(ConfidenceState,
                  CONFIDENT,
                  nomadz_msgs::msg::SideConfidence::confidence_state,
                  confidenceState), /**< The state of confidence */
});

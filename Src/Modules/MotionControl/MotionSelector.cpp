/**
 * @file MotionSelector.cpp
 *
 * This file implements a module that is responsible for controlling the motion.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>, <A
 * href="mailto:allli@tzi.de">Alexander Härtl</A> and Jesse Richter-Klug
 */

#include <algorithm>
#include "MotionSelector.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Range.h"

MAKE_MODULE(MotionSelector, Motion Control)

PROCESS_WIDE_STORAGE(MotionSelector) MotionSelector::theInstance = 0;

void MotionSelector::stand() {
  if (theInstance) {
    theInstance->forceStand = true;
  }
}

void MotionSelector::move() {
  if (theInstance) {
    theInstance->forceStand = false;
  }
}

void MotionSelector::update(MotionSelection& motionSelection) {
  static int interpolationTimes[MotionRequest::numOfMotions];
  interpolationTimes[MotionRequest::walk] = 600;
  interpolationTimes[MotionRequest::kick] = 600;
  interpolationTimes[MotionRequest::specialAction] = 100;

  static const int playDeadDelay(2000);

  if (lastExecution) {
    MotionRequest::Motion requestedMotion = theMotionRequest.motion;
    SpecialActionRequest specialActionReq = theMotionRequest.specialActionRequest;

    if (enableFallProtection &&
        (lastMotion != MotionRequest::specialAction ||
         (lastMotion == MotionRequest::specialAction && theSpecialActionsOutput.isLeavingPossible &&
          theRobotInfo.penalty == PENALTY_NONE)) &&
        theFallDownState.state == FallDownState::falling) {
      requestedMotion = MotionRequest::specialAction;
      specialActionReq.mirror = false;
      switch (theFallDownState.direction) {
      case FallDownState::back:
        specialActionReq.specialAction = SpecialActionRequest::fallprotectionback;
        break;
      case FallDownState::front:
        specialActionReq.specialAction = SpecialActionRequest::fallprotectionfront;
        break;
      case FallDownState::left:
      case FallDownState::right:
        specialActionReq.specialAction = SpecialActionRequest::fallprotectionside;
        break;
      default:
        requestedMotion = theMotionRequest.motion;
        specialActionReq = theMotionRequest.specialActionRequest;
        break;
      }
    } else if (requestedMotion == MotionRequest::walk && theFallDownState.state == FallDownState::upright &&
               (!theGroundContactState.contact && theGroundContactState.fsr)) {
      requestedMotion = MotionRequest::specialAction;
      specialActionReq.specialAction = SpecialActionRequest::stand;
      specialActionReq.mirror = false;
    }

    if ((lastMotion == MotionRequest::walk) && (requestedMotion == MotionRequest::walk)) {
      motionSelection.walkRequest = theMotionRequest.walkRequest;
    } else {
      motionSelection.walkRequest = WalkRequest();
    }

    // check if the target motion can be the requested motion (mainly if leaving is possible)
    if ((lastMotion == MotionRequest::walk &&
         (theWalkingEngineOutput.isLeavingPossible || !theGroundContactState.contact)) ||
        (lastMotion == MotionRequest::specialAction && theSpecialActionsOutput.isLeavingPossible) ||
        (lastMotion == MotionRequest::kick && theKickEngineOutput.isLeavingPossible) ||
        (lastMotion == MotionRequest::walk &&
         theFallDownState.state == FallDownState::falling)) // never immediatly leave kick or get up
    {
      motionSelection.targetMotion = requestedMotion;
    }

    if (requestedMotion == MotionRequest::specialAction) {
      motionSelection.specialActionRequest = specialActionReq;
    } else {
      motionSelection.specialActionRequest = SpecialActionRequest();
      if (motionSelection.targetMotion == MotionRequest::specialAction) {
        motionSelection.specialActionRequest.specialAction = SpecialActionRequest::numOfSpecialActionIDs;
      }
    }

    const bool afterPlayDead(prevMotion == MotionRequest::specialAction &&
                             lastActiveSpecialAction == SpecialActionRequest::playDead);
    const int bodyInterpolationTime(afterPlayDead ? playDeadDelay : interpolationTimes[motionSelection.targetMotion]);
    interpolate(motionSelection.ratios, MotionRequest::numOfMotions, bodyInterpolationTime, motionSelection.targetMotion);

    if (motionSelection.ratios[MotionRequest::specialAction] < 1.f) {
      if (motionSelection.targetMotion == MotionRequest::specialAction) {
        motionSelection.specialActionMode = MotionSelection::first;
      } else {
        motionSelection.specialActionMode = MotionSelection::deactive;
      }
    } else {
      motionSelection.specialActionMode = MotionSelection::active;
    }

    if (motionSelection.specialActionMode == MotionSelection::active &&
        motionSelection.specialActionRequest.specialAction != SpecialActionRequest::numOfSpecialActionIDs) {
      lastActiveSpecialAction = motionSelection.specialActionRequest.specialAction;
    }
  }

  if (lastMotion != motionSelection.targetMotion) {
    prevMotion = lastMotion;
  }

  lastMotion = motionSelection.targetMotion;

  PLOT("module:MotionSelector:ratios:walk", motionSelection.ratios[MotionRequest::walk]);
  PLOT("module:MotionSelector:ratios:specialAction", motionSelection.ratios[MotionRequest::specialAction]);
  PLOT("module:MotionSelector:lastMotion", lastMotion);
  PLOT("module:MotionSelector:prevMotion", prevMotion);
  PLOT("module:MotionSelector:targetMotion", motionSelection.targetMotion);

  lastExecution = theFrameInfo.time;

#ifndef NDEBUG
  const Rangef& ratioLimits = Rangef::ZeroOneRange();
  for (int i = 0; i < MotionRequest::numOfMotions; ++i) {
    ASSERT(ratioLimits.isInside(motionSelection.ratios[i]));
  }
#endif
}

void MotionSelector::interpolate(float* ratios, const int amount, const int interpolationTime, const int targetMotion) {
  // increase / decrease all ratios according to target motion
  const unsigned deltaTime(theFrameInfo.getTimeSince(lastExecution));
  float delta(static_cast<float>(deltaTime) / interpolationTime);
  ASSERT(SystemCall::getMode() == SystemCall::logfileReplay || delta > 0.00001f);
  float sum(0);
  for (int i = 0; i < amount; i++) {
    if (i == targetMotion) {
      ratios[i] += delta;
    } else {
      ratios[i] -= delta;
    }
    ratios[i] = std::max(ratios[i], 0.0f); // clip ratios
    sum += ratios[i];
  }
  ASSERT(sum != 0);
  // normalize ratios
  for (int i = 0; i < amount; i++) {
    ratios[i] /= sum;
    if (std::abs(ratios[i] - 1.f) < 0.00001f) {
      ratios[i] =
        1.f; // this should fix a "motionSelection.ratios[motionSelection.targetMotion] remains smaller than 1.f" bug
    }
  }
}

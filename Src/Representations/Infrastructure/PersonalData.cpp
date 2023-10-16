/**
 * @file PersonalData.cpp
 *
 * Declaration of a class representing information about the Robot.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include "PersonalData.h"

PersonalDataCompressed::PersonalDataCompressed(const PersonalData& personalData)
    : ballLockState(static_cast<char>(personalData.ballLockState)), ballScore(personalData.ballScore) {}

PersonalDataCompressed::operator PersonalData() const {
  PersonalData personalData;
  personalData.syncTeamRequired = false;
  personalData.hasBallLock = (ballLockState == PersonalData::HAS_LOCK);
  personalData.ballLockState = static_cast<PersonalData::BallLockState>(ballLockState);
  personalData.ballScore = ballScore;
  return personalData;
}

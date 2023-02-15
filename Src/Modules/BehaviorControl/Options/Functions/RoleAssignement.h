/**
 * @file RoleAssignement.h
 *
 * All functions presented here have to do with role assignement.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

void updateMyDistanceToBall() {
  thePersonalData.myDistanceToBall = (theCombinedWorldModel.ballState.position - theRobotPose.translation).abs();
}

/* return the index of another robot with a specific role (excluding ourself).
 * first found is returned. This is based on the assumption that maximal 2 robots
 * have the same role.
 * returns -1 if none is found
 */
int getIndexOfOtherRole(BehaviorStatus::Role role) {
  for (int i = 0; i < TeamMateData::numOfPlayers; i++) {
    if (theTeamMateData.behaviorStatus[i].role == role && theRobotInfo.number != i) {
      return i;
    }
  }
  return -1;
}

/* get the total number of robots with a specific role (including ourself) */
int getNumberOfRoles(BehaviorStatus::Role role) {
  int count = 0;
  for (int i = 0; i < TeamMateData::numOfPlayers; i++) {
    if (theTeamMateData.behaviorStatus[i].role == role) {
      ++count;
    }
  }
  // theTeamMateData does not contain our data, so check this separately
  if (theBehaviorStatus.role == role) {
    ++count;
  }
  return count;
}

// ---------------Decisions--------------------
/* check whether I am closer to the ball than another role */
bool amICloserToBallThan(BehaviorStatus::Role role = BehaviorStatus::striker) {
  updateMyDistanceToBall();
  int myDistanceToBall = thePersonalData.myDistanceToBall;
  int otherRoleIdx = getIndexOfOtherRole(role);
  if (otherRoleIdx == -1) {
    return true;
  }
  int otherDistanceToBall = theTeamMateData.robotsPersonalData[otherRoleIdx].myDistanceToBall;

  // 300 subtracted to avoid oscillation
  if (myDistanceToBall < (otherDistanceToBall - 300.f)) {
    return true;
    // If the last striker has been penalized or is fallen over,
    // do not trust his last sent data. You are now the closest player!
  } else if (!theTeamMateData.isFullyActive[otherRoleIdx]) {
    return true;
  } else {
    return false;
  }
}

bool amIClosestToBall() {
  updateMyDistanceToBall();
  bool amIClosest = true;
  int otherDistanceToBall;
  int myDistanceToBall = (theCombinedWorldModel.ballState.position - theRobotPose.translation).abs();
  for (int i = 2; i <= 5; i++) {
    if (i != theRobotInfo.number) {
      // otherDistanceToBall = theTeamMateData.robotsPersonalData[i].myDistanceToBall;
      otherDistanceToBall = (theCombinedWorldModel.ballState.position - theTeamMateData.robotPoses[i].translation).abs();
      if (myDistanceToBall > otherDistanceToBall) {
        amIClosest = false;
      }
    }
  }

  if (amIClosest) {
    return true;
  } else {
    return false;
  }
}

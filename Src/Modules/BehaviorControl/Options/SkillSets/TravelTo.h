/**
 * @file TravelTo.h
 *
 * Walk to some point and heading on the field, as quickly as possible, while avoiding other players.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(TravelTo, Pose2D target) {

  Pose2D waypoint;

  initial_state(walk) {
    transition { waypoint = getWaypoint(target); }
    action {
      if (waypoint == target) {
        WalkTo(waypoint);
      } else {
        WalkTo(waypoint, false);
      }
    }
  }
}

struct Obstacle {
  Vector2<> relObstaclePos;
  Vector2<> globalObstaclePos;
  Vector2<> relAvoidPos;
  Vector2<> globalAvoidPos;
  float distToPath;
  bool leftOfPath;
};

Pose2D getWaypoint(const Pose2D& target) {
  float distAvoidanceThreshold = 500.f; //[mm]
  float avoidanceOffset = 350.f;        // [mm]
  float targetOffset = 250.f;           // [mm] -> to avoid slowing down
  float slowDownAvoidOffset = 200.f;
  float stoppingDist = 200.f;
  bool waypointOutsideOfFieldCheck = true;
  float waypointOutsideOfFieldOffset = 50.f;
  float ignoreObstacle = 1000;
  Pose2D waypoint = target;

  std::vector<Obstacle> obstacles;
  obstacles.clear();

  Vector2<> startPosTravelPath = Vector2<>(theRobotPose.translation.x, theRobotPose.translation.y);
  Vector2<> endPosTravelPath = Vector2<>(target.translation.x, target.translation.y);

  for (std::vector<PlayerModel::Player>::const_iterator p = thePlayerModel.players.begin();
       p != thePlayerModel.players.end();
       ++p) {
    Obstacle obstacle;

    obstacle.relObstaclePos = p->relPosOnField;
    obstacle.globalObstaclePos = theRobotPose * p->relPosOnField;
    obstacle.leftOfPath = false; // Just initialized here, updated in shortestDistPointLineSeg
    bool obstacleAlongPath = true;
    obstacle.distToPath = shortDistPointLineSeg(
      startPosTravelPath, endPosTravelPath, obstacle.globalObstaclePos, obstacle.leftOfPath, obstacleAlongPath);

    if (obstacleAlongPath && (p->relPosOnField).abs() <= ignoreObstacle) {
      obstacle.relAvoidPos = p->relPosOnField;
      obstacle.relAvoidPos.x += targetOffset;

      if (obstacle.leftOfPath) {
        obstacle.relAvoidPos.y -= avoidanceOffset;
        obstacle.globalAvoidPos = theRobotPose * obstacle.relAvoidPos;

        // Check if waypoint is outside of field -> if yes, switch it to the other side
        if (isWaypointOutsideOfField(obstacle.globalAvoidPos, waypointOutsideOfFieldOffset) && waypointOutsideOfFieldCheck) {
          obstacle.leftOfPath = false;
          obstacle.relAvoidPos.y += 2 * avoidanceOffset;
          obstacle.globalAvoidPos = theRobotPose * obstacle.relAvoidPos;
        }

      } else {
        obstacle.relAvoidPos.y += avoidanceOffset;
        obstacle.globalAvoidPos = theRobotPose * obstacle.relAvoidPos;

        // Check if waypoint is outside of field -> if yes, switch it to the other side
        if (isWaypointOutsideOfField(obstacle.globalAvoidPos, waypointOutsideOfFieldOffset) && waypointOutsideOfFieldCheck) {
          obstacle.leftOfPath = true;
          obstacle.relAvoidPos.y -= 2 * avoidanceOffset;
          obstacle.globalAvoidPos = theRobotPose * obstacle.relAvoidPos;
        }
      }

      // if(theRobotInfo.number == 1) {
      //     std::cout << "@> " << obstacle.relAvoidPos.x << " " << obstacle.relAvoidPos.y << " " <<
      //     obstacle.relObstaclePos.x << std::endl;
      // }

      obstacles.push_back(obstacle);
    }
  }

  Vector2<> waypointVec;
  waypointVec.x = target.translation.x;
  waypointVec.y = target.translation.y;

  unsigned int numOfObst = (unsigned int)obstacles.size();

  if (numOfObst > 0) {
    Vector2<> relAvoidPos;
    Obstacle closestObstacle;
    float smallestDistance = 100000.0;
    int i = 0;
    int indexClosestObstacle = -1;
    for (auto& obstacle : obstacles) {
      if (obstacle.relObstaclePos.abs() < smallestDistance && obstacle.distToPath <= distAvoidanceThreshold) {
        smallestDistance = obstacle.relObstaclePos.abs();
        closestObstacle = obstacle;
        indexClosestObstacle = i;
      }
      i++;
    }

    if (indexClosestObstacle != -1) {
      if (numOfObst == 1) {
        waypointVec = theRobotPose * closestObstacle.relAvoidPos;
        relAvoidPos = closestObstacle.relAvoidPos;
      } else {
        bool foundFreePath = false;
        i = 0;
        while (!foundFreePath) {
          bool leftFree = true;
          bool rightFree = true;
          int obs_counter = 0;
          for (auto& obstacle : obstacles) {
            if ((i == 0 && obs_counter == indexClosestObstacle) || (i > 0 && obs_counter == i - 1)) {
              continue;
            }
            // Check if obstacle within 2 * avoidanceOffset of left/right of closestObstacle
            if (obstacle.relObstaclePos.y > (closestObstacle.relObstaclePos.y - 2 * avoidanceOffset) &&
                obstacle.relObstaclePos.y < closestObstacle.relObstaclePos.y) {
              leftFree = false;
            }
            if (obstacle.relObstaclePos.y < (closestObstacle.relObstaclePos.y + 2 * avoidanceOffset) &&
                obstacle.relObstaclePos.y > closestObstacle.relObstaclePos.y) {
              rightFree = false;
            }
            obs_counter++;
          }
          relAvoidPos = closestObstacle.relObstaclePos;
          if (leftFree || rightFree) {
            relAvoidPos.x += targetOffset;
            if (closestObstacle.leftOfPath && rightFree) {
              relAvoidPos.y += avoidanceOffset;
            } else if (!closestObstacle.leftOfPath && leftFree) {
              relAvoidPos.y -= avoidanceOffset;
            } else if (rightFree) {
              relAvoidPos.y += avoidanceOffset;
            } else {
              relAvoidPos.y -= avoidanceOffset;
            }
            foundFreePath = true;
          } else {
            // Take next Obstacle if both right and left of the closest Obstacle are occupied
            if (i == indexClosestObstacle && i < numOfObst - 1) {
              i++;
            } else if (i == numOfObst) {
              relAvoidPos = closestObstacle.relAvoidPos;
              break;
            }
            closestObstacle = obstacles.at(i);
            i++;
          }
        }
        waypointVec = theRobotPose * relAvoidPos;
      }
    }

    waypoint.translation.x = waypointVec.x;
    waypoint.translation.y = waypointVec.y;
    float alpha = atan2f(target.translation.y - waypoint.translation.y, target.translation.x - waypoint.translation.x);
    waypoint.rotation = alpha;
    // Avoid slowing down by setting new target before reaching point
    float distFromWayPoint =
      sqrt((theRobotPose.translation.x - waypoint.translation.x) * (theRobotPose.translation.x - waypoint.translation.x) +
           (theRobotPose.translation.y - waypoint.translation.y) * (theRobotPose.translation.y - waypoint.translation.y));
    // std::cout << "distFromWayPoint: " << distFromWayPoint << std::endl;
    if (distFromWayPoint < slowDownAvoidOffset) {
      waypoint = target;
      // std::cout << "TARGET!" << std::endl;
    }
    for (auto& obstacle : obstacles) {
      if (obstacle.relObstaclePos.abs() < stoppingDist) {
        waypoint.translation.x = theRobotPose.translation.x;
        waypoint.translation.y = theRobotPose.translation.y;
        if (obstacle.leftOfPath) {
          waypoint.rotation = theRobotPose.rotation - 0.1f;
        } else {
          waypoint.rotation = theRobotPose.rotation + 0.1f;
          // std::cout << "Too close!!" << obstacle.relObstaclePos.abs() << std::endl;
        }
      }
    }
    // std::cout << "Orig. target :" << targetRel.x << " y: " << targetRel.y << std::endl;
    // std::cout << "Obstacles" << numOfObst << std::endl;
    // std::cout << "waypoint x:" << relAvoidPos.x << " y: " << relAvoidPos.y << std::endl;
  }
  return waypoint;
}

bool isWaypointOutsideOfField(Vector2<> globalAvoidPos, float waypointOutsideOfFieldOffset) {
  // xAxis check -> only x-axis for now (for goal posts)
  if ((globalAvoidPos.x > (theFieldDimensions.xPosOpponentGroundline + waypointOutsideOfFieldOffset)) ||
      (globalAvoidPos.x < (-1.f * theFieldDimensions.xPosOpponentGroundline - waypointOutsideOfFieldOffset))) {
    return true;
  }

  // yAxis check
  // if((globalAvoidPos.y > (theFieldDimensions.yPosLeftSideline + waypointOutsideOfFieldOffset)) || (globalAvoidPos.y <
  // (-1.f*theFieldDimensions.yPosLeftSideline - waypointOutsideOfFieldOffset))){
  //   return true;
  // }

  return false;
}

float shortDistPointLineSeg(
  Vector2<> startPointLine, Vector2<> endPointLine, Vector2<> point, bool& leftOfLine, bool& obstacleAlongPath) {
  leftOfLine = true;
  obstacleAlongPath = true;
  float diffX = endPointLine.x - startPointLine.x;
  float diffY = endPointLine.y - startPointLine.y;
  if ((diffX == 0) && (diffY == 0)) {
    diffX = point.x - startPointLine.x;
    diffY = point.y - startPointLine.y;
    return sqrt(diffX * diffX + diffY * diffY);
  }

  float t = ((point.x - startPointLine.x) * diffX + (point.y - startPointLine.y) * diffY) / (diffX * diffX + diffY * diffY);

  if (t < 0) {
    // point is nearest to the first point
    diffX = point.x - startPointLine.x;
    diffY = point.y - startPointLine.y;
    obstacleAlongPath = false;
  } else if (t > 1) {
    // point is nearest to the end point i.e x2 and y2
    diffX = point.x - endPointLine.x;
    diffY = point.y - endPointLine.y;
    obstacleAlongPath = false;
  } else {
    // if perpendicular line intersect the line segment.
    diffX = point.x - (startPointLine.x + t * diffX);
    diffY = point.y - (startPointLine.y + t * diffY);
  }

  // Determine if obstacle is on the right or the left side of the line
  float side = -(endPointLine.x - startPointLine.x) * (point.y - startPointLine.y) +
               (endPointLine.y - startPointLine.y) * (point.x - startPointLine.x);
  if (side > 0) {
    leftOfLine = false;
  }

  // returning shortest distance
  return sqrt(diffX * diffX + diffY * diffY);
}

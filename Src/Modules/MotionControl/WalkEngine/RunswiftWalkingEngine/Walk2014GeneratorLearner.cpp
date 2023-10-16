/**
 * @file Walk2014GeneratorLearner.cpp
 *
 * This file declares a module that learns joint offsets for the walking.
 *
 * This file is subject to the terms of the BHuman 2017 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Philip Reichenberg
 */

#include "Walk2014GeneratorLearner.h"
#include <algorithm>

MAKE_MODULE(Walk2014GeneratorLearner, Motion Control);

Walk2014GeneratorLearner::Walk2014GeneratorLearner() {
  gyroForwardBalanceFactor = -1.f;
  gyroBackwardBalanceFactor = -1.f;
  oldForwardGyro = 0.f;
  oldAverageMaxGyro = 0.f;
  oldAverageMaxGyroPhase1 = 0.f;
  oldBackwardGyro = 0.f;
  oldAverageMinGyro = 9000.f;
  oldAverageMinGyroPhase1 = 9000.f;
  phaseLearn = 0;
  learnIteration = 0;
  isForwardPhase = false;
  wasPositiv = false;
}

void Walk2014GeneratorLearner::update(WalkLearner& walkLearner) {
  // set functions and copy the balance factors from the walkGenerator
  if (gyroForwardBalanceFactor >= 0.f) {
    walkLearner.useWalkLearner = true;
  }
  walkLearner.setBaseWalkParams = [this](float gyroForward, float gyroBackward, float speedTransX) {
    setBaseWalkParams(gyroForward, gyroBackward, speedTransX);
  };
  walkLearner.newGyroBackwardBalance = gyroBackwardBalanceFactor;
  walkLearner.newGyroForwardBalance = gyroForwardBalanceFactor;
  learnGyroBalanceFactor(walkLearner);
}

void Walk2014GeneratorLearner::setBaseWalkParams(float gyroForward, float gyroBackward, float speedTransX) {
  if (gyroForwardBalanceFactor < 0.f) {
    gyroForwardBalanceFactor = gyroForward;
    gyroBackwardBalanceFactor = gyroBackward;
  }
  speed = speedTransX;
}

void Walk2014GeneratorLearner::learnGyroBalanceFactor(WalkLearner& walkLearner) {

  // std::cout << "Inertia gyro y: " << theInertiaSensorData.gyro.y << std::endl;
  // std::cout << learnIteration << std::endl;
  // std::cout << isForwardPhase << std::endl;
  // std::cout << wasPositiv << std::endl;
  if (gyroForwardBalanceFactor < 0.f) {
    return;
  }
  // numOfGyro was changed. Init the variables.
  if (theWalk2014Modifier.numOfGyroPeaks != static_cast<int>(gyroForwardMax.size()) ||
      theWalk2014Modifier.numOfGyroPeaks != static_cast<int>(gyroBackwardMin.size())) {
    gyroForwardMax = std::vector<float>(theWalk2014Modifier.numOfGyroPeaks, 0.f);
    gyroBackwardMin = std::vector<float>(theWalk2014Modifier.numOfGyroPeaks, 0.f);
    learnIteration = 0;
    isForwardPhase = true;
    wasPositiv = true;
  }

  // If numOfGyro is > 0 and the robot walks over 100mm/s, then go into the learning part
  if (theWalk2014Modifier.numOfGyroPeaks > 0 && speed > 30.f) {
    // std::cout << "gyro y " << theInertiaSensorData.gyro.y << " gyroForwardBalanceFactor " << gyroForwardBalanceFactor
    //           << std::endl;
    // We are sampling over the positiv gyro peaks
    if (isForwardPhase && wasPositiv) {
      if (theInertiaSensorData.gyro.y > gyroForwardMax[learnIteration]) {
        gyroForwardMax[learnIteration] = theInertiaSensorData.gyro.y;
      } else if (theInertiaSensorData.gyro.y < 0.f) {
        isForwardPhase = !isForwardPhase;
      }
    }
    // We are sampling over the negativ gyro peaks
    else if (!isForwardPhase && !wasPositiv) {
      if (theInertiaSensorData.gyro.y < gyroBackwardMin[learnIteration]) {
        gyroBackwardMin[learnIteration] = theInertiaSensorData.gyro.y;
      } else if (theInertiaSensorData.gyro.y > 0.f) {
        isForwardPhase = !isForwardPhase;
      }
      if (learnIteration + 1 <= theWalk2014Modifier.numOfGyroPeaks && isForwardPhase) {
        learnIteration += 1;
        // std::cout << "positive gyro peak " << gyroForwardMax[learnIteration] << " neg. gyro peak " <<
        // gyroBackwardMin[learnIteration] << std::endl;
      }
    }

    // This is needed to be sure, that we collect a negativ gyro peak after a positiv gyro peak was collected and the
    // sign has changed. Same vice versa.
    else {
      wasPositiv =
        (isForwardPhase && theInertiaSensorData.gyro.y > 0.f) || (!isForwardPhase && theInertiaSensorData.gyro.y < 0.f)
          ? !wasPositiv
          : wasPositiv;
    }

    // We collected enough gyro peaks. Rate them and adjust gyroBalanceFactor
    if (learnIteration >= theWalk2014Modifier.numOfGyroPeaks) {
      learnIteration = 0;
      float number = 0;
      if (phaseLearn > 0 && phaseLearn < 3) {
        number = 1.1f;
      } else if (phaseLearn > 4 || phaseLearn == 0) {
        phaseLearn = 0;
        number = 2.f * randomFloat(); // there is definitely a better solution to do random deciding
        // which gyro shall get changed next. But I, Philip, am too lazy for it
        // right now.
      }
      float average = 0.f;
      float averageDif = 0.f;

      // std::cout << "average " << average << " averageDiff " << averageDif << std::endl;
      // std::cout << "number " << number << std::endl;
      // std::cout << "phaseLearn " << phaseLearn << std::endl;
      // calculate average value and average deviation (standard deviation could be tested?)
      // averageDif will get used to change the gyros for balancing
      // Change gyroForwardBalanceFactor
      if (number > 1.f) {
        for (float f : gyroForwardMax) {
          average += f;
        }
        if (average != 0.f) {
          average = average / theWalk2014Modifier.numOfGyroPeaks;
        }

        for (float f : gyroForwardMax) {
          averageDif += (f - average) * (f - average);
        }
        if (averageDif != 0.f) {
          averageDif = averageDif / theWalk2014Modifier.numOfGyroPeaks;
        }

        // Rate gyro peaks without change
        if (phaseLearn == 0) {
          phaseLearn += 1;
          oldForwardGyro = gyroForwardBalanceFactor;
          gyroForwardBalanceFactor += averageDif * theWalk2014Modifier.balanceChangeFactor;
          gyroForwardBalanceFactor = std::max(gyroForwardBalanceFactor, 0.f);
          oldAverageMaxGyro = average * averageDif;
        }
        // Rate gyro peaks with first change
        else if (phaseLearn == 1) {
          phaseLearn += 1;
          gyroForwardBalanceFactor += 2.f * (oldForwardGyro - gyroForwardBalanceFactor);
          gyroForwardBalanceFactor = std::max(gyroForwardBalanceFactor, 0.f);
          oldAverageMaxGyroPhase1 = average * averageDif;
        }
        // Rate gyro peaks with second change and take the best version of all 3.
        else {
          // std::cout << oldAverageMaxGyro << "  " << oldAverageMaxGyroPhase1 << "  " << average * averageDif << std::endl;
          phaseLearn = 10;
          if (oldAverageMaxGyro < oldAverageMaxGyroPhase1 && oldAverageMaxGyro <= average * averageDif) {
            gyroForwardBalanceFactor = oldForwardGyro;
          } else if (oldAverageMaxGyro > oldAverageMaxGyroPhase1 && oldAverageMaxGyro > average * averageDif) {
            gyroForwardBalanceFactor += 2.f * (oldForwardGyro - gyroForwardBalanceFactor);
          }

          // std::cout << "new gyroforward balancefactor : " <<   gyroForwardBalanceFactor << std::endl;
        }
      }
      // Change gyroBackwardBalanceFactor
      else {
        for (float f : gyroBackwardMin) {
          average += f;
        }
        if (average != 0.f) {
          average = average / theWalk2014Modifier.numOfGyroPeaks;
        }

        for (float f : gyroBackwardMin) {
          averageDif += (f - average) * (f - average);
        }
        if (averageDif != 0.f) {
          averageDif = averageDif / theWalk2014Modifier.numOfGyroPeaks;
        }

        // Rate gyro peaks without change
        if (phaseLearn == 0) {
          phaseLearn = 3;
          oldBackwardGyro = gyroBackwardBalanceFactor;
          gyroBackwardBalanceFactor += averageDif * theWalk2014Modifier.balanceChangeFactor;
          gyroBackwardBalanceFactor = std::max(gyroBackwardBalanceFactor, 0.f);
          oldAverageMinGyro = average * averageDif;
        }
        // Rate gyro peaks with first change
        else if (phaseLearn == 3) {
          phaseLearn += 1;
          gyroBackwardBalanceFactor += 2.f * (oldBackwardGyro - gyroBackwardBalanceFactor);
          gyroBackwardBalanceFactor = std::max(gyroBackwardBalanceFactor, 0.f);
          oldAverageMinGyroPhase1 = average * averageDif;
        }
        // Rate gyro peaks with second change and take the best version of all 3.
        else {
          phaseLearn = 10;
          if (oldAverageMinGyro < oldAverageMinGyroPhase1 && oldAverageMinGyro <= average * averageDif) {
            gyroBackwardBalanceFactor = oldBackwardGyro;
          } else if (oldAverageMinGyro > oldAverageMinGyroPhase1 && oldAverageMinGyro > average * averageDif) {
            gyroBackwardBalanceFactor += 2.f * (oldBackwardGyro - gyroBackwardBalanceFactor);
          }
        }
      }
      // reset list with the saved gyro peaks.
      gyroForwardMax = std::vector<float>(theWalk2014Modifier.numOfGyroPeaks, 0.f);
      gyroBackwardMin = std::vector<float>(theWalk2014Modifier.numOfGyroPeaks, 0.f);
    }
  }
}

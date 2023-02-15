/**
 * @file GroundContactDetector.h
 *
 * Declaration of module GroundContactDetector.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/MotionControl/MotionInfo.h"

MODULE(GroundContactDetector)
REQUIRES(RobotModel)
REQUIRES(FrameInfo)
REQUIRES(MotionRequest)
REQUIRES(SensorData)
USES(TorsoMatrix)
USES(MotionInfo)
PROVIDES_WITH_MODIFY(GroundContactState)
LOADS_PARAMETER(bool, useFSR)
LOADS_PARAMETER(float, fsrContactChangeTime)
LOADS_PARAMETER(float, fsrMinContact);
LOADS_PARAMETER(float, noContactMinAccNoise)
LOADS_PARAMETER(float, noContactMinGyroNoise)
LOADS_PARAMETER(float, contactMaxAngleNoise)
LOADS_PARAMETER(float, contactAngleActivationNoise)
LOADS_PARAMETER(float, contactMaxAccZ) // deactivate
END_MODULE

/**
 * @class GroundContactDetector
 * A module for sensor data filtering.
 */
class GroundContactDetector : public GroundContactDetectorBase {
public:
  /** Default constructor. */
  GroundContactDetector();

private:
  bool contact;                  /**< Whether the robot has ground contact or not */
  unsigned int contactStartTime; /**< Time when the robot started having ground contact */
  bool useAngle;                 /**< Whether the estimated angle will be used to detect ground contact loss */

  RotationMatrix expectedRotationInv;

  /**
   * An averager for computing the average of up to \c n entries.
   * Accumulating an error will be avoided by gradually recounting the sum of all added entries.
   */
  template <typename C, int n, typename T = float> class Averager {
  public:
    Averager() { clear(); }

    void clear() {
      oldSum = newSum = C();
      pos = count = 0;
    }

    void add(const C& val) {
      unsigned int posRaw = pos + 1;
      pos = posRaw % n;
      if (count == n) {
        if (posRaw == n) {
          oldSum = newSum;
          newSum = C();
        }
        oldSum -= data[pos];
      } else {
        ++count;
      }
      data[pos] = val;
      newSum += val;
    }

    bool isFull() const { return count == n; }
    C getAverage() const { return (oldSum + newSum) / T(count); }

  private:
    C data[n];
    C oldSum;
    C newSum;
    unsigned int pos;
    unsigned int count;
  };

  Averager<Vector2<>, 60> angleNoises;
  Averager<Vector3<>, 60> accNoises;
  Averager<Vector2<>, 60> gyroNoises;
  Averager<Vector3<>, 60> accValues;
  Averager<Vector2<>, 60> gyroValues;
  Averager<float, 5> calibratedAccZValues;

  unsigned int lastContact;
  unsigned int lastNonContact;

  /**
   * Updates the GroundContactState representation .
   * @param groundContactState The ground contact representation which is updated by this module.
   */
  void update(GroundContactState& groundContactState);
};

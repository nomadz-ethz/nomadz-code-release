/**
 * @file FLIPMObserver.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:arne.moos@tu-dortmund.de> Arne Moos</a>
 */

#pragma once
/* tells the RingBuffer to check the boundaries */
#define LIMIT_CHECK

#include <list>
#include <stdio.h>
#include "Core/Enum.h"
#include "Core/System/File.h"
#include "Core/Module/Module.h"
#include "Core/RingBufferNew.h"
#include "Core/Math/Vector.h"
#include "Core/Math/Vector2.h"
#include "Representations/ROSRepresentationHelpers.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/ObservedFLIPMError.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Representations/Sensing/ZMPModel.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/Footpositions.h"

#include <Eigen/Eigen>

#define COMMA ,
const int static_N = 50; /**< Length of the preview phase */

MODULE(FLIPMObserver)
REQUIRES(WalkingEngineParams)
REQUIRES(ActualCoM)
REQUIRES(ActualCoMFLIPM)
REQUIRES(PatternGenRequest)
REQUIRES(InertiaSensorData)
REQUIRES(ZMPModel)
REQUIRES(FootSteps)
REQUIRES(RobotModel)
REQUIRES(RobotDimensions)
REQUIRES(Footpositions)
USES(WalkingInfo)
USES(TargetCoM)
PROVIDES_WITH_MODIFY(ObservedFLIPMError)
END_MODULE

STREAMABLE_DECLARE(FLIPMObserverParams)
class FLIPMObserverParams : public FLIPMObserverParamsBaseWrapper {
public:
  FLIPMObserverParams() {
    FILE* stream;
    std::string name = "flipmObserverParams";
    std::string fullPath;
    std::list<std::string> names = File::getFullNames(name + ".cfg");
    bool found = false;

    for (auto& path : names) {
      stream = fopen(path.c_str(), "r");
      if (stream) {
        found = true;
        fullPath = path;
        break;
      }
    }
    if (!found) {
      fullPath = File::getBHDir() + std::string("/Config/Robots/Default/") + name + std::string(".cfg");
      ASSERT(false);
    }
    fclose(stream);
    InMapFile file(name + std::string(".cfg"));
    if (file.exists()) {
      file >> *this;
    } else {
      OUTPUT_TEXT("Warning, could not create" + fullPath);
    }
  }

  ENUM(CoMProvider, targetCoM, MRE_CoM, IMU_CoM);

  ENUM(ACCProvider, targetAcc, ZMP_Acc, ZMP_CoM1_Acc, IMU_Acc);

  bool activateSensorX;
  float sensorXFactor;
  float sensorControlRatioObserverX[3];
  bool activateSensorY;
  float sensorYFactor;
  float sensorControlRatioObserverY[3];
  float maxDiffCoMClip;
  float maxDiffACCClip;
  CoMProvider CoM1Provider;
  ACCProvider ACC1Provider;
  CoMProvider CoM2Provider;
  unsigned int CoM1Delay;
  float CoM1OffsetX;
  float CoM1OffsetY;

  unsigned int CoM2Delay;
  float CoM2OffsetX;
  float CoM2OffsetY;
  float CoM2DiffXOffset;
  float CoM2DiffYOffset;

  float accFilter;
  bool useRCS;

  void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(activateSensorX);
    STREAM(sensorXFactor);
    STREAM(sensorControlRatioObserverX);
    STREAM(activateSensorY);
    STREAM(sensorYFactor);
    STREAM(sensorControlRatioObserverY);
    STREAM(maxDiffCoMClip);
    STREAM(maxDiffACCClip);
    STREAM(CoM1Provider);
    STREAM(ACC1Provider);
    STREAM(CoM2Provider);
    STREAM(CoM1Delay);
    STREAM(CoM1OffsetX);
    STREAM(CoM1OffsetY);
    STREAM(CoM2Delay);
    STREAM(CoM2OffsetX);
    STREAM(CoM2OffsetY);
    STREAM(CoM2DiffXOffset);
    STREAM(CoM2DiffYOffset);
    STREAM(accFilter);
    STREAM(useRCS);
    STREAM_REGISTER_FINISH;
  }
};
INSTANTIATE_ROS_HELPERS(FLIPMObserverParams)

class FLIPMObserver : public FLIPMObserverBase {
  STREAMABLE(FLIPMObsvX,
             {
               ,
               (int)N,
               (float)z_h,
               (float)g,
               (Matrix6x3f)L,
             });

  STREAMABLE(FLIPMObsvY,
             {
               ,
               (int)N,
               (float)z_h,
               (float)g,
               (Matrix6x3f)L,
             });

  FLIPMObsvX paramsFLIPMObsvX;
  FLIPMObsvY paramsFLIPMObsvY;

public:
  FLIPMObserver() : counter(0), isStable(true), localSensorScale(0), filteredAccX(0.0), filteredAccY(0.0) {

    InMapFile xCfg("FLIPMObsvX.cfg");
    xCfg >> paramsFLIPMObsvX;

    InMapFile yCfg("FLIPMObsvY.cfg");
    yCfg >> paramsFLIPMObsvY;
  };

  void update(ObservedFLIPMError& observedFLIPMError);

protected:
  FLIPMObserverParams flipmObserverParams;

private:
  RingBufferNew<Vector2<>, static_N> accDelayBuffer, /**< Buffer to deal with the sensor delay. */
    coM1DelayBuffer,                                 /**< Buffer to deal with the sensor delay. */
    coM2DelayBuffer,                                 /**< Buffer to deal with the sensor delay. */
    realCoM1DelayBuffer, realCoM2DelayBuffer;
  int counter;
  bool isStable;
  float localSensorScale;
  Point lastRealZMP;

  float filteredAccX;
  float filteredAccY;
};

/**
 * @file NaoKinematic.h
 *
 * This file implements the inverse kinematic.
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original authors are <A href="mailto:Stefan.Czarnetzki@uni-dortmund.de">Stefan Czarnetzki</A>,
 * <ahref="mailto:oliver.urbann@tu-dortmund.de"> Oliver Urbann</a>
 */

#pragma once
#include <fstream>

#include "Representations/MotionControl/KinematicOutput.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Modules/MotionControl/WalkEngine/DortmundWalkingEngine/PatternGenerator.h"
#include "Core/Math/Matrix.h"

#ifndef WALKING_SIMULATOR
#include "Core/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/Sensing/RobotModel.h"
#include "Core/RingBufferWithSumNew.h"

MODULE(NaoKinematic)
REQUIRES(RobotDimensions)
REQUIRES(KinematicRequest)
PROVIDES_WITH_MODIFY(KinematicOutput)
LOADS_PARAMETER(bool, useBHKinematics)
END_MODULE

class CombinedIterativeNaoKinematic;

class RCXPKinematicInterface : public Streamable {
public:
  JointRequest jointRequest;
  double kinematicRequest[12];
  bool calculated;

  RCXPKinematicInterface() {
    calculated = false;

    for (int i = 0; i < 12; i++) {
      kinematicRequest[i] = 0;
    }
  }

  virtual void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(jointRequest)
    STREAM(kinematicRequest)
    STREAM(calculated)
    STREAM_REGISTER_FINISH;
  }
};

#else
#include "math/Vector3_D.h"
#include "bhumanstub.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Configuration/RobotDimensions.h"

/**
 * @class Robot
 * Base class for NaoKinematic. This is just a stub to be compatible to
 * BHuman framework, where the data members are defined in the blackboard.
 */
class NaoKinematicBase {
public:
  /** The desired foot positions. */
  KinematicRequest theKinematicRequest;

  /** Actual angles. */
  JointData theJointData;

  /** Dimensions of the robot. */
  RobotDimensions theRobotDimensions;
};

#endif

/**
 * @class Robot
 * The inverse kinematic for the Nao robot.
 */
class NaoKinematic : public NaoKinematicBase {
public:
  friend class CombinedIterativeNaoKinematic;

  /** Constructor */
  NaoKinematic();
  /** Desctructor */
  ~NaoKinematic();

  /**
   * Calculates the angles the first leg. It depends on the request
   * which leg is the first leg. E.g. in single support phase the standing
   * leg is the first leg.
   * \param whichSideJoint0 Index of first joint of leg.
   * \param position The desired foot position.
   * \param rotation The desired foot rotation.
   * \param jointRequest Filled with the calculated angles.
   */
  static bool calcLegJoints(JointData::Joint whichSideJoint0,
                            const Vector3<>& position,
                            const Vector3<>& rotation,
                            JointRequest& jointRequest,
                            const RobotDimensions& robotDimensions);

  /** This method implemenets a mixed kinematic. Here you can set t0, given by the calcLegJoints, to be
   * compatible to the real Joint 0 (where the angle of both legs has to be equal)
   * This resuts in a z rotation of the foot.
   * \param whichSideJoint0 Index of first joint of leg.
   * \param position The desired foot position.
   * \param rotation The desired foot rotation. z is ignored here.
   * \param jointRequest Filled with the calculated angles.
   */
  static bool calcLegJoints(JointData::Joint whichSideJoint0,
                            const Vector3<>& position,
                            const Vector3<>& rotation,
                            float t0,
                            JointRequest& jointRequest,
                            const RobotDimensions& robotDimensions);

  /**
   * Calculates the angles from the given foot positions in theKinematicRequest.
   * \param walkingEngineOutput Filled with the angles.
   */
  void update(KinematicOutput& walkingEngineOutput);
#if 0
   void update(RawKinematicOutput& rawKinematicOutput);

   RCXPKinematicInterface rcxpKinematic;
#endif
private:
  Vector3<> checkConstraints(Vector3<> lf, float lfr, Vector3<> rf, float rfr, bool left);
  std::ofstream logfile;
};
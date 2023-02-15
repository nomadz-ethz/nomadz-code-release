/**
 * @file MassCalibration.h
 *
 * Declaration of a class for representing the relative positions and masses of mass points.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
 */

#pragma once

#include "Core/Math/SpatialInertia.h"
#include "Core/Math/Vector3.h"
#include "Core/Enum.h"
#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/mass_calibration.hpp"
#endif
STREAMABLE_DECLARE(MassCalibration)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/mass_info.hpp"
#endif
STREAMABLE_DECLARE(MassInfo)

STREAMABLE_ROS(MassCalibration, {
public:
  ENUM(Limb,
       neck,
       head,
       shoulderLeft,
       bicepsLeft,
       elbowLeft,
       foreArmLeft,
       handLeft,
       shoulderRight,
       bicepsRight,
       elbowRight,
       foreArmRight,
       handRight,
       pelvisLeft,
       hipLeft,
       thighLeft,
       tibiaLeft,
       ankleLeft,
       footLeft,
       pelvisRight,
       hipRight,
       thighRight,
       tibiaRight,
       ankleRight,
       footRight,
       torso);

  float totalMass = 5305.39f; /**< The total mass of the Robot (in g). */

  /**
   * Information on the mass distribution of a limb of the robot.
   */
  STREAMABLE_ROS(MassInfo, {
    public :,
    FIELD_WRAPPER_DEFAULT(float, nomadz_msgs::msg::MassInfo::mass, mass), /**< The mass of this limb. */
    FIELD_WRAPPER_DEFAULT(Vector3<>,
                          nomadz_msgs::msg::MassInfo::offset,
                          offset), /**< The offset of the center of mass of this limb relative to its hinge. */
    FIELD_WRAPPER_DEFAULT(float[6],
                          nomadz_msgs::msg::MassInfo::inertia_matrix_l_t,
                          inertiaMatrixLT), /**< The moment of inertia matrix of this limb. */

    /**
     * Default constructor.
     */
  }),
    FIELD_WRAPPER_DEFAULT(MassInfo[numOfLimbs],
                          nomadz_msgs::msg::MassCalibration::masses,
                          masses), /**< Information on the mass distribution of all joints. */
});

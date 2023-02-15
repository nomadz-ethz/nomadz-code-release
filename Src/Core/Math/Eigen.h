/**
 * @file Eigen.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in License.Nao_Devils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#pragma once

#include "Core/System/BHAssert.h" // Our Eigen extensions use ASSERT

// Extend the Eigen classes with our own methods (see: http://eigen.tuxfamily.org/dox-devel/TopicCustomizingEigen.html)
#define EIGEN_MATRIXBASE_PLUGIN "Tools/Math/EigenMatrixBaseExtensions.h"
#define EIGEN_ARRAY_PLUGIN "Tools/Math/EigenArrayExtensions.h"

#ifdef WINDOWS
#define WARN_UNUSED_RESULT
#else
#define WARN_UNUSED_RESULT __attribute__((warn_unused_result))
#endif

#include <Eigen/Dense>
#include "Core/Streams/Eigen.h"

class Angle;
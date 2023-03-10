/**
 * @file FieldBoundaryProvider.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Alexis Tsogias
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ColorReference.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Modeling/Odometer.h"

MODULE(FieldBoundaryProvider)
REQUIRES(BodyContour)
REQUIRES(CameraInfo)
REQUIRES(CameraMatrix)
REQUIRES(ColorReference)
REQUIRES(Image)
REQUIRES(ImageCoordinateSystem)
REQUIRES(Odometer)
PROVIDES_WITH_DRAW(FieldBoundary)
DEFINES_PARAMETER(int, scanlienDistance, 8)
DEFINES_PARAMETER(int, upperBound, 2)
DEFINES_PARAMETER(int, lowerBound, 5)
DEFINES_PARAMETER(int, nearVertJump, 4)
DEFINES_PARAMETER(int, farVertJump, 2)
DEFINES_PARAMETER(int, nonGreenPenalty, 2)
DEFINES_PARAMETER(int, nonGreenPenaltyDistance, 3500)
DEFINES_PARAMETER(int, minGreenCount, 5)
END_MODULE

/**
 *
 */
class FieldBoundaryProvider : public FieldBoundaryProviderBase {
private:
  struct BoundaryScanline {
    int x;
    int yStart;
    int yMax;
    int score;
    int maxScore;
    const Image::Pixel* pImg;
  };

  typedef FieldBoundary::InImage InImage;
  typedef FieldBoundary::InField InField;

  InField lowerCamConvexHullOnField;
  InImage lowerCamSpotsInImage;
  InImage lowerCamSpostInterpol;

  void update(FieldBoundary& fieldBoundary);

  void handleLowerCamSpots();
  void findBundarySpots(FieldBoundary& fieldBoundary, int horizon);

  bool cleanupBoundarySpots(InImage& boundarySpots) const;
  std::vector<InImage> calcBoundaryCandidates(InImage boundarySpots) const;
  void
  findBestBoundary(const std::vector<InImage>& boundaryCandidates, const InImage& boundarySpots, InImage& boundary) const;

  inline bool isLeftOf(const Vector2<int>& a, const Vector2<int>& b, const Vector2<int>& c) const;
  InImage getUpperConvexHull(const InImage& boundary) const;

  int clipToBoundary(const InImage& boundary, int x) const;
};

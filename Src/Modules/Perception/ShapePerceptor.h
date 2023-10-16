/**
 * @file ShapePerceptor.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Core/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ShapePercept.h"
#include "Representations/Perception/PlayerPercept.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Perception/PenaltyMarkPercept.h"
#include "Core/Debugging/DebugImages.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"

MODULE(ShapePerceptor)
REQUIRES(BodyContour)
REQUIRES(CameraInfo)
REQUIRES(CameraMatrix)
REQUIRES(FieldDimensions)
REQUIRES(Image)
REQUIRES(PlayerPercept)
REQUIRES(FieldBoundary)
PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallSpots)
PROVIDES_WITH_MODIFY_AND_OUTPUT(PenaltyMarkPercept)
PROVIDES_WITH_MODIFY_AND_OUTPUT(ShapePercept)
LOADS_PARAMETER(unsigned char, imgChannel) /**< 0 for cb, 1 for y, 2 for cr, 3 for h, 4 for s, 5 for i */
LOADS_PARAMETER(unsigned char, drawWhich)  /**< image, downsized, gradX, gradY, gradL, gradA, peaks, lineH, lineHPeaks,
                                              fieldLineH, peaksNoLines, circleHough, circlePeaks */
LOADS_PARAMETER(float, maxRange)           /**<  reliable detections are not to be expected beyond this distance (in mm) */
LOADS_PARAMETER(bool, clipBodyContour)     /**< do not process anything inside body contour */
LOADS_PARAMETER(double, peakThreshold)     /**< lower gives more edges, higher gives less */
LOADS_PARAMETER(unsigned char,
                lineASpread) /**< uncertainty involving gradient angles for line detection (in 8-bit degrees) */
LOADS_PARAMETER(unsigned char, lineRSpread)   /**< uncertainty involving gradient radius for line detection (in px) */
LOADS_PARAMETER(bool, voteUsingRatios)        /**< "vertical" pixels get more votes */
LOADS_PARAMETER(unsigned short, lineMinVotes) /**< lower gives more lines, higher gives less */
LOADS_PARAMETER(bool, sortLinesFirst)         /**< whether to process most-voted lines first, or don't care */
LOADS_PARAMETER(
  unsigned short,
  maxLines) /**< max number of lines to consider; use with sortLinesFirst to keep top x most-voted lines only */
LOADS_PARAMETER(
  unsigned char,
  lineMinHits) /**< minimum consecutive gradient peaks for a line segment to start; higher gives shorter segments */
LOADS_PARAMETER(
  unsigned char,
  lineMaxMisses) /**< maximum consecutive absent peaks for a line segment to end; higher gives longer segments */
LOADS_PARAMETER(bool, newSegmenter)    /**< use new line segmentation code */
LOADS_PARAMETER(unsigned char, labels) /**< label lines with... 0: none, 1: number of votes, 2: number of hits */
LOADS_PARAMETER(
  unsigned char,
  oldLineMinHits) /**< !old! minimum consecutive gradient peaks for a line segment to start; higher gives shorter segments */
LOADS_PARAMETER(
  unsigned char,
  oldLineMinMisses) /**< !old! minimum consecutive absent peaks for a line segment to end; higher gives shorter segments */
LOADS_PARAMETER(unsigned int, circMaxIter) /**< upper bound to number of peaks to process for circle detection */
LOADS_PARAMETER(unsigned char,
                circASpread) /**< uncertainty involving gradient angles for circle detection (in 8-bit degrees) */
LOADS_PARAMETER(unsigned char, circRSpread)    /**< uncertainty involving ball radius for circle detection (in px) */
LOADS_PARAMETER(unsigned char, circMinVotes)   /**< lower gives more circles, higher gives less */
LOADS_PARAMETER(unsigned char, ballMinR)       /**< assume ball radius must be larger than this (in px) */
LOADS_PARAMETER(unsigned char, ballCandidates) /**< number of peaks to keep & evaluate */
LOADS_PARAMETER(float, maxRangePenaltyMark)    /**< reliable penalty marks not to be expected beyond this distance (in mm) */
LOADS_PARAMETER(float, sizePenaltyMark)        /**< size of the penalty mark (in one directon in mm) */
LOADS_PARAMETER(unsigned char, whiteThresh)    /**< penalty mark: min value of image for pixel to be considered white */
LOADS_PARAMETER(unsigned char,
                edgeThresh) /**< penalty mark: min value of edge peak for pixel to be considered distinct enough edge */

LOADS_PARAMETER(float, ballPlayerAllowedPercentage) /**< whether to draw balls detected in the image */
LOADS_PARAMETER(bool, drawBalls)                    /**< whether to draw balls detected in the image */
LOADS_PARAMETER(bool, drawLines)                    /**< whether to draw line segments detected in the image */
LOADS_PARAMETER(bool, drawImage)                    /**< draw anything at all */
END_MODULE

class ShapePerceptor : public ShapePerceptorBase {
public:
  ShapePerceptor() {}

  void init();

  void update(BallSpots& ballSpots);
  void update(PenaltyMarkPercept& penaltyMarkPercept);
  void update(ShapePercept& shapePercept);

  DECLARE_DEBUG_IMAGE(shapes);

private:
#ifdef TARGET_SIM
  const unsigned char downFactor = 2; // 1 is no scaling, 2 is half, and so on
#endif
#ifdef TARGET_ROBOT
  const unsigned char downFactor = 2; // 1 is no scaling, 2 is half, and so on
#endif

  PenaltyMarkPercept* thePenaltyMarkPercept = NULL;

  unsigned char angle_LUT[256];
  float sin_LUT[256 * 3];
  float cos_LUT[256 * 3];
  char sin_r_LUT[256 * 3][96]; // Unlikely a ball is bigger than 96 pixels
  char cos_r_LUT[256 * 3][96];
  unsigned char angle2_LUT[256][256]; // use as angle2_LUT[gy][gx]

  static const int ratio_LUT_interval = 6;
  static const int hMax = 480;
  unsigned char ratio_LUT[hMax / ratio_LUT_interval + 1];

  void downsize(unsigned char* downsized, const unsigned int w, const unsigned int h, const unsigned int yFar);

  void grad(const unsigned char* downsized,
            signed char* gradX,
            signed char* gradY,
            unsigned char* gradL,
            unsigned char* gradA,
            const unsigned int w,
            const unsigned int h,
            const unsigned char frame,
            const unsigned int yStart);

  void peaks(const unsigned char* gradL,
             const unsigned char* gradA,
             unsigned char* grid,
             std::vector<int>& list,
             const unsigned int w,
             const unsigned int h,
             const unsigned int yStart,
             const unsigned char frame);

  void allLines(const std::vector<int>& peakList,
                const unsigned char* gradA,
                unsigned short* lineHough,
                const unsigned int w,
                const unsigned int h,
                const unsigned int lineR,
                const unsigned int lineRMax,
                const unsigned int lineA,
                unsigned char lineAOffset);

  void findLinePeaks(const unsigned short* lineHough,
                     unsigned short* linePeaks,
                     std::vector<unsigned int>& linePeakList,
                     const unsigned int lineR,
                     const unsigned int lineA,
                     const unsigned char lineAOffset);

  void excludeSimilarLines(const unsigned short* linePeaks,
                           const std::vector<unsigned int>& linePeakList,
                           unsigned short* filteredPeaks,
                           std::vector<unsigned int>& scratchlist,
                           const unsigned int lineR);

  void segmentLines(const unsigned short* lineVotes,
                    const std::vector<unsigned int>& fieldLineList,
                    const unsigned char* gradA,
                    const unsigned char* edges,
                    std::vector<ShapePercept::LineSegment>& segments,
                    unsigned char* linesErased,
                    const unsigned int w,
                    const unsigned int h,
                    const unsigned int lineR,
                    const unsigned int lineRMax,
                    const unsigned int lineA,
                    const unsigned char lineAOffset);

  void eraseNearSegment(const ShapePercept::LineSegment& segment,
                        unsigned char* grid,
                        const unsigned int w,
                        const unsigned int h);

  void listNonzero(const unsigned char* edges,
                   std::vector<int>& edgeList,
                   const unsigned int w,
                   const unsigned int h,
                   const unsigned int yStart,
                   const unsigned char frame);

  void allCircles(const std::vector<int>& peakList,
                  const unsigned char* gradA,
                  unsigned char* circleHough,
                  const unsigned int w,
                  const unsigned int h);

  void findBalls(const std::vector<int>& edgeList,
                 const unsigned char* circleHough,
                 std::vector<std::pair<unsigned int, unsigned char>>& circleList,
                 unsigned char* circlePeaks,
                 const unsigned int w,
                 const unsigned int h,
                 const unsigned int yStart,
                 const unsigned char frame);

  void drawTheImage(const void* grid,
                    const int pxType,
                    const unsigned int drawW,
                    const unsigned int drawH,
                    const unsigned int w,
                    const unsigned int h,
                    const unsigned int yStart,
                    const bool isOptical,
                    const unsigned char bias);

  void drawTheBalls(const unsigned char* circleHough,
                    const unsigned int drawW,
                    const unsigned int drawH,
                    const unsigned int w,
                    const unsigned int h,
                    const bool isOptical);

  void drawTheLines(const std::vector<ShapePercept::LineSegment>& segments,
                    const unsigned int drawW,
                    const unsigned int drawH,
                    const unsigned int w,
                    const unsigned int h,
                    const bool isOptical);

  void findBrightEdges(const unsigned char* downsized,
                       const unsigned char* edges,
                       const unsigned char* gradA,
                       unsigned char* brightEdges,
                       unsigned char whiteThresh,
                       unsigned char edgeThresh,
                       unsigned int w,
                       unsigned int h,
                       unsigned int yStart,
                       unsigned char frame);

  void eraseInsideCircles(const std::vector<std::pair<unsigned int, unsigned char>>& circleList,
                          const unsigned char* brightEdges,
                          unsigned char* edgesNoCircles,
                          unsigned int w,
                          unsigned int h,
                          unsigned char frame);

  // void findPenaltyMark(const unsigned char* edgesNoCircles, bool& seen, Vector2<int>& position,
  //       unsigned int w, unsigned int h, unsigned int yTop, unsigned char frame);

  void findPenaltyMark(const unsigned char* edgesNoCircles,
                       const unsigned char* edges,
                       bool& seen,
                       Vector2<int>& position,
                       unsigned int w,
                       unsigned int h,
                       int yTop,
                       unsigned char frame,
                       int size);

  inline bool ballRadiusInImage(const int y, const int downFactor, int& radius) const;
  bool ballRadiusInImage(const Vector2<int>& centerInImageInt, const int downFactor, int& radius) const;

  std::vector<unsigned int> ballList;

  // These go into PenaltyMarkPercept.
  // The duplication of information is unfortunate: the module system requires each representation to have
  // its own update function, but most of the heavy work is done only inside update(ShapePercept)
  bool penaltyMarkSeen;
  Vector2<int> penaltyMarkPos; // in downsized image coordinates
};

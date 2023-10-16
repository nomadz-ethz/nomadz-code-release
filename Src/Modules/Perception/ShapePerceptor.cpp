/**
 * @file ShapePerceptor.cpp
 *
 * @note Thanks to John Morrison (Northern Bites)
 * http://www.bowdoin.edu/~robocup/media/papers/theses/JohnMorrison_Honors_updated.pdf
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <random>
#include <utility>
#include <vector>
#include "ShapePerceptor.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Debugging/DebugImages.h"
#include "Core/Debugging/Debugging.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"
#include "../Util/vectorclass/vectorclass.h"

MAKE_MODULE(ShapePerceptor, Perception)

static unsigned char getBinaryAngle(const unsigned char* LUT, const signed short gx, const signed short gy);
static inline unsigned char angleDifference(const unsigned char a1, const unsigned char a2);
static void compute_LUTs(unsigned char angle_LUT[256],
                         float sin_LUT[256 * 3],
                         float cos_LUT[256 * 3],
                         char sin_r_LUT[256 * 3][96],
                         char cos_r_LUT[256 * 3][96],
                         unsigned char angle2_LUT[256][256]);

void ShapePerceptor::init() {}

void ShapePerceptor::update(BallSpots& ballSpots) {
  const unsigned int w = theCameraInfo.width / downFactor;
  ballSpots.ballSpots.clear();
  for (int i = 0; i < (int)ballList.size(); ++i) {
    const unsigned int x = ballList[i] % w;
    const unsigned int y = ballList[i] / w;

    int r;
    if (ballRadiusInImage(Vector2<int>(x, y), downFactor, r) && r > ballMinR) {
      ballSpots.addBallSpot(r * downFactor, x * downFactor, y * downFactor);
    }
  }
}

void ShapePerceptor::update(PenaltyMarkPercept& penaltyMarkPercept) {
  // penaltyMarkSeen & penaltyMarkPos come from update(ShapePercept)
  thePenaltyMarkPercept = &penaltyMarkPercept;
}

void ShapePerceptor::update(ShapePercept& shapePercept) {
  unsigned char scratchpad[320 * 320] = {};
  unsigned char downsized[320 * 240] = {};

  if (theCameraInfo.width <= 0 || theCameraInfo.height <= 0) {
    return;
  }

  const unsigned int camW = theCameraInfo.width;
  const unsigned int camH = theCameraInfo.height;
  const unsigned int downW = camW / downFactor;
  const unsigned int downH = camH / downFactor;

  unsigned int yFar = 0;
  {
    Vector2<> farthestPointOnField(maxRange, 0);
    const float pan = std::atan2(theCameraMatrix.rotation.c0.y, theCameraMatrix.rotation.c0.x);
    farthestPointOnField.rotate(pan);
    Vector2<int> farthestPointInImage(-1, -1);
    if (Geometry::calculatePointInImage(farthestPointOnField, theCameraMatrix, theCameraInfo, farthestPointInImage)) {
      yFar = std::max<int>(0, farthestPointInImage.y - ballMinR);
    }
  }

  STOP_TIME_ON_REQUEST("ShapePerceptor:downsize", { downsize(downsized, downW, downH, yFar); });

  // Prepare lookup tables for (gy/gx => angle) & for sin/cos on 8-bit angles
  static bool LUT_computed = false;
  if (!LUT_computed) {
    compute_LUTs(angle_LUT, sin_LUT, cos_LUT, sin_r_LUT, cos_r_LUT, angle2_LUT);
    LUT_computed = true;
  }

  signed char gradX[320 * 240] = {};
  signed char gradY[320 * 240] = {};
  unsigned char gradL[320 * 240] = {};
  unsigned char gradA[320 * 240] = {}; // 8-bit angles

  const unsigned char frame = 1;                                       // Area to leave empty near border
  const unsigned int yStart = std::max<int>(frame, yFar / downFactor); // In downsized px

  shapePercept.cutoffY = yStart * downFactor;

  STOP_TIME_ON_REQUEST("ShapePerceptor:grad", {
    // downsized => gradX, gradY, gradL, gradA
    grad(downsized, gradX, gradY, gradL, gradA, downW, downH, frame, yStart);
  });

  unsigned char edges[320 * 240];
  std::vector<int> peakList;
  STOP_TIME_ON_REQUEST("ShapePerceptor:peaks", {
    // gradL, gradA => edges, peakList
    memset(edges, 0, 320 * 240);
    peakList.clear();
    peaks(gradL, gradA, edges, peakList, downW, downH, yStart, frame);
  });

#ifdef TARGET_SIM
  const unsigned int lineRMax = 160; // max magnitude of R
#endif
#ifdef TARGET_ROBOT
  const unsigned int lineRMax = 160; // max magnitude of R
#endif

  // Prepare lookup table for ratios, indexed on y coordinate
  STOP_TIME_ON_REQUEST("ShapePerceptor:lut", {
    if (voteUsingRatios) {
      for (unsigned int i = 0, n = downH / ratio_LUT_interval; i <= n; ++i) {
        float ratio = -1.f;
        if (Geometry::estimateInvPixelRatioOnField(
              (i + 0.5f) * ratio_LUT_interval * downFactor, theCameraMatrix, theCameraInfo, ratio)) {
          ratio_LUT[i] = std::max<unsigned char>((unsigned char)round(ratio), 1);
        } else {
          ratio_LUT[i] = 1;
        }
      }
    }
  });

  const unsigned char lineAOffset = 10;
  const unsigned int lineR = 2 * lineRMax;          // also w
  const unsigned int lineA = 256 + 2 * lineAOffset; // also h
  unsigned short lineHough[lineR * lineA];
  const unsigned short* lineHoughD = lineHough + lineR * lineAOffset;
  STOP_TIME_ON_REQUEST("ShapePerceptor:allLines", {
    // peakList, gradA => lineHough
    memset(lineHough, 0, sizeof(lineHough));
    allLines(peakList, gradA, lineHough, downW, downH, lineR, lineRMax, lineA, lineAOffset);
  });

  // Find local maxima in an 8-neighborhood in circleHough
  unsigned short linePeaks[lineR * lineA]; // 320 x 276; angle offset by lineAOffset
  const unsigned short* linePeaksD = linePeaks + lineR * lineAOffset;
  unsigned short filteredPeaks[lineR * lineA];
  const unsigned short* filteredPeaksD = filteredPeaks + lineR * lineAOffset;
  std::vector<unsigned int> linePeakList;
  std::vector<unsigned int> filteredPeakList;
  STOP_TIME_ON_REQUEST("ShapePerceptor:filterLines", {
    // lineHough => linePeaks, linePeakList
    memset(linePeaks, 0, sizeof(linePeaks));
    linePeakList.clear();
    findLinePeaks(lineHough, linePeaks, linePeakList, lineR, lineA, lineAOffset);

    // linePeaks, linePeakList => filteredPeaks, filteredPeakList
    memset(filteredPeaks, 0, sizeof(filteredPeaks));
    excludeSimilarLines(linePeaks, linePeakList, filteredPeaks, filteredPeakList, lineR);
  });

  std::vector<ShapePercept::LineSegment> segmentList;
  unsigned char gradPNoLines[320 * 240];
  STOP_TIME_ON_REQUEST("ShapePerceptor:segmentLines", {
    // Sort line list by highest votes first
    // filteredPeakList, lineHough => filteredPeakList
    if (sortLinesFirst) {
      std::sort(filteredPeakList.begin(), filteredPeakList.end(), [&lineHough](unsigned int p1, unsigned int p2) {
        return lineHough[p1] > lineHough[p2];
      });
    }

    // filteredPeaks, filteredPeakList, gradA, edges => segmentList, gradPNoLines
    segmentList.clear();
    memset(gradPNoLines, 0, 320 * 240);
    segmentLines(filteredPeaks,
                 filteredPeakList,
                 gradA,
                 edges,
                 segmentList,
                 gradPNoLines,
                 downW,
                 downH,
                 lineR,
                 lineRMax,
                 lineA,
                 lineAOffset);

    // segmentList => shapePercept.segments
    shapePercept.segments.clear();
    for (unsigned int i = 0; i < segmentList.size(); ++i) {
      ShapePercept::LineSegment output = segmentList[i];
      output.p1 *= downFactor;
      output.p2 *= downFactor;

      shapePercept.segments.push_back(output);
    }
  });

  std::vector<int> curveList; // list of indices of peaks (with lines removed)
  STOP_TIME_ON_REQUEST("ShapePerceptor:listCurves", {
    // gradPNoLines => curveList
    curveList.clear();
    listNonzero(gradPNoLines, curveList, downW, downH, yStart, frame);
  });

  unsigned char circleHough[320 * 240];
  STOP_TIME_ON_REQUEST("ShapePerceptor:allCircles", {
    // curveList, gradA => circleHough
    memset(circleHough, 0, downW * downH);
    allCircles(curveList, gradA, circleHough, downW, downH);
  });

  // Find local maxima in a 24-neighborhood in circleHough
  unsigned char circlePeaks[320 * 240];
  std::vector<std::pair<unsigned int, unsigned char>> circleList; // first: location index, second: hough votes
  STOP_TIME_ON_REQUEST("ShapePerceptor:findBalls", {
    // curveList, circleHough => circleList, circlePeaks
    memset(circlePeaks, 0, 320 * 240);
    ballList.clear();
    findBalls(curveList, circleHough, circleList, circlePeaks, downW, downH, yStart, frame);
  });

  unsigned int yStartPenaltyMark = 0; // TODO Make this (and other unsigned ints) signed
  {
    Vector2<> farthestPointOnField(maxRangePenaltyMark * cosf(theCameraMatrix.rotation.getZAngle()),
                                   maxRangePenaltyMark * sinf(theCameraMatrix.rotation.getZAngle()));
    Vector2<int> farthestPointInImage(-1, -1);
    if (Geometry::calculatePointInImage(farthestPointOnField, theCameraMatrix, theCameraInfo, farthestPointInImage)) {
      yStartPenaltyMark = std::max<int>(0, farthestPointInImage.y / downFactor);
    }
  }

  // Find the penalty mark
  unsigned char brightEdges[320 * 240];
  STOP_TIME_ON_REQUEST("ShapePerceptor:findBrightEdges", {
    memset(brightEdges, 0, downW * downH);

    // downsized, gradPNoLines, gradA => brightEdges
    findBrightEdges(
      downsized, gradPNoLines, gradA, brightEdges, whiteThresh, edgeThresh, downW, downH, yStartPenaltyMark, frame);
  });

  // Erase circles from bright edges
  unsigned char edgesNoCircles[320 * 240];
  STOP_TIME_ON_REQUEST("ShapePerceptor:eraseInsideCircles", {
    // circleList, brightEdges => edgesNoCircles
    eraseInsideCircles(circleList, brightEdges, edgesNoCircles, downW, downH, frame);
  });

  STOP_TIME_ON_REQUEST("ShapePerceptor:findPenaltyMark", {
    // edgesNoCircles => penaltyMarkSeen, penaltyMarkPos
    findPenaltyMark(edgesNoCircles,
                    edges,
                    penaltyMarkSeen,
                    penaltyMarkPos,
                    downW,
                    downH,
                    (int)yStartPenaltyMark,
                    frame,
                    (int)sizePenaltyMark);
    if (thePenaltyMarkPercept) {
      thePenaltyMarkPercept->seen = penaltyMarkSeen;
      if (thePenaltyMarkPercept->seen) {
        thePenaltyMarkPercept->positionInImage = penaltyMarkPos * int(downFactor);
        Geometry::calculatePointOnField(thePenaltyMarkPercept->positionInImage,
                                        theCameraMatrix,
                                        theCameraInfo,
                                        thePenaltyMarkPercept->relativePositionOnField);
      }
      thePenaltyMarkPercept->draw();
    }
  });

  const unsigned char* gradXD = (unsigned char*)gradX;
  const unsigned char* gradYD = (unsigned char*)gradY;

  unsigned int drawWs[] = {
    0, downW, downW, downW, downW, downW, downW, lineR, lineR, lineR, downW, downW, downW, downW, downW};
  unsigned int drawHs[] = {0, downH, downH, downH, downH, downH, downH, 256, 256, 256, downH, downH, downH, downH, downH};
  const void* grids[] = {NULL,
                         downsized,
                         gradXD,
                         gradYD,
                         gradL,
                         gradA,
                         edges,
                         lineHoughD,
                         linePeaksD,
                         filteredPeaksD,
                         gradPNoLines,
                         circleHough,
                         circlePeaks,
                         brightEdges,
                         edgesNoCircles};
  const int pxTypes[] = {1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1};
  //          drawWhich = {   0,         1,      2,      3,     4,     5,     6,          7,          8,              9,
  //          10,          11,          12,          13,             14};
  drawWs[0] = camW;
  drawHs[0] = camH;

  STOP_TIME_ON_REQUEST("ShapePerceptor:draw", {
    const unsigned char maxDrawings = 15;
    const unsigned int drawW = (drawWhich < maxDrawings) ? drawWs[drawWhich] : downW;
    const unsigned int drawH = (drawWhich < maxDrawings) ? drawHs[drawWhich] : downH;
    const void* grid = (drawWhich < maxDrawings) ? grids[drawWhich] : scratchpad;
    const int pxType = pxTypes[drawWhich];
    const bool isOptical = (drawW * (camW / drawW) == (unsigned int)camW && drawH * (camH / drawH) == (unsigned int)camH);
    const unsigned char bias = (drawWhich == 2 || drawWhich == 3) ? 128 : 0;

    drawTheImage(grid, pxType, drawW, drawH, downW, downH, yFar, isOptical, bias);
    drawTheBalls(circleHough, drawW, drawH, downW, downH, isOptical);
    drawTheLines(segmentList, drawW, drawH, downW, downH, isOptical);

    // Handle interaction (click on Hough image)
    // TODO Put this into its own function
    Vector2<int> point(-1, -1);
    MODIFY("module:CameraCalibrator:point", point); // HACK Using CameraCalibrator to handle clicks
    if (drawWhich >= 7 && drawWhich <= 9 && point.x >= 0 && point.x < (int)lineR && point.y >= 0 && point.y < 256) {
      CROSS("module:ShapePerceptor:lines", point.x, point.y, 1, 1, Drawings::ps_solid, ColorClasses::red);
      DRAWTEXT("module:ShapePerceptor:lines",
               point.x,
               point.y,
               9,
               ColorClasses::red,
               "r = " << (point.x - (signed)lineR / 2) << ", a = " << point.y);

      const unsigned char a = (unsigned char)point.y;
      const float r = point.x - (int)lineRMax;
      const float cosA = cos_LUT[a + 256];
      const float sinA = sin_LUT[a + 256];

      const signed int xTop =
        (signed int)std::round((r - (float)((float)1 - (float)downH / 2) * sinA) / cosA + (float)downW / 2);
      const signed int xBottom =
        (signed int)std::round((r - (float)((float)-1 + (float)downH / 2) * sinA) / cosA + (float)downW / 2);
      const signed int yLeft =
        (signed int)std::round((r - (float)((float)1 - (float)downW / 2) * cosA) / sinA + (float)downH / 2);
      const signed int yRight =
        (signed int)std::round((r - (float)((float)-1 + (float)downW / 2) * cosA) / sinA + (float)downH / 2);

      Vector2<int> p1(0, yLeft);
      Vector2<int> p2(downW, yRight);
      if (sinA == 0) { // Perfectly vertical
        p1.x = xTop;
        p1.y = 0;
        p2.x = xBottom;
        p2.y = downH;
      }

      LINE("module:ShapePerceptor:lines",
           p1.x,
           p1.y + (256 - downH) / 2,
           p2.x,
           p2.y + (256 - downH) / 2,
           1,
           Drawings::ps_solid,
           ColorClasses::red);
    }
    // Show pixel aspect ratio on click
    if (((drawWhich >= 1 && drawWhich <= 6) || drawWhich == 10) && point.x >= 0 && point.x < (int)downW && point.y >= 0 &&
        point.y < (int)downH) {
      CROSS("module:ShapePerceptor:lines", point.x, point.y, 1, 1, Drawings::ps_solid, ColorClasses::yellow);

      float ratio = -1.f;
      float ratioEst = -1.f;
      Geometry::estimateInvPixelRatioOnField((float)point.y * downFactor, theCameraMatrix, theCameraInfo, ratioEst);
      if (Geometry::calculatePixelRatioOnField(
            (float)point.x * downFactor, (float)point.y * downFactor, theCameraMatrix, theCameraInfo, ratio))
        DRAWTEXT("module:ShapePerceptor:lines",
                 point.x,
                 point.y,
                 9,
                 ColorClasses::green,
                 round(1.f / ratio * 10.f) / 10.f << " - " << round(ratioEst * 10.f) / 10.f << " = "
                                                  << round((1.f / ratio - ratioEst) * 100.f) / 100.f);
      else
        DRAWTEXT("module:ShapePerceptor:lines",
                 point.x,
                 point.y,
                 9,
                 ColorClasses::red,
                 round(1.f / ratio * 10.f) / 10.f << " - " << round(ratioEst * 10.f) / 10.f << " = "
                                                  << round((1.f / ratio - ratioEst) * 100.f) / 100.f);
    }
  });
}

// Downsizes theImage by a factor of ShapePerceptor::downFactor
void ShapePerceptor::downsize(unsigned char* downsized,
                              const unsigned int w,
                              const unsigned int h,
                              const unsigned int yFar) {
  // HACK This assumes the Image representation is structured as an array
  const int widthStep = theImage[1] - theImage[0];
  // Number of chars (typeof(downsized)) between a pixel and the one to its right (on the downsized image)
  const int xStep = downFactor * sizeof(Image::Pixel) / sizeof(unsigned char);
  // Number of chars (typeof(downsized)) between a pixel on the right edge and a pixel on the left edge in the next row (on
  // the downsized image)
  const int yStep = xStep * (widthStep - w);
  // Number of chars (typeof(downsized)) between a pixel and the one to its right (on the original image)
  const int imageRight = sizeof(Image::Pixel) / sizeof(unsigned char);
  // Number of chars (typeof(downsized)) between a pixel and the one below it (on the original image)
  const int imageDown = widthStep * sizeof(Image::Pixel) / sizeof(unsigned char);
  const int yStart = std::max<int>(yFar - downFactor, 0);
  const unsigned char* chan; // Pointer to 1st pixel in a specific channel
  switch (imgChannel) {
  case 0:
    chan = &(theImage[(yStart / downFactor) * downFactor][0].cb);
    for (unsigned int i = yStart / downFactor, idx = i * w; i < h; ++i, chan += yStep) {
      for (unsigned int j = 0; j < w; ++j, idx += 1, chan += xStep) {
        downsized[idx] = *chan;
      }
    }
    break;
  case 1:
    chan = &(theImage[(yStart / downFactor) * downFactor][0].y);
    for (unsigned int i = yStart / downFactor, idx = i * w; i < h; ++i, chan += yStep) {
      for (unsigned int j = 0; j < w; ++j, idx += 1, chan += xStep) {
        // Area-average downsampling on the original image (assumes downFactor = 2)
        // This channel has extra division by two: Y (luma) changes more drastically than the other channels
        downsized[idx] = ((*chan + *(chan + imageRight) + *(chan + imageDown) + *(chan + imageRight + imageDown)) >> 2) >> 2;
      }
    }
    break;
  case 2:
    chan = &(theImage[(yStart / downFactor) * downFactor][0].cr);
    for (unsigned int i = yStart / downFactor, idx = i * w; i < h; ++i, chan += yStep) {
      for (unsigned int j = 0; j < w; ++j, idx += 1, chan += xStep) {
        downsized[idx] = *chan;
      }
    }
    break;
  case 3: // Hue
  case 4: // Saturation
  case 5: // Intensity
    for (unsigned int y = yStart; y < h; ++y) {
      for (unsigned int x = 0; x < w; ++x) {
        unsigned char c[3];
        Image::Pixel px = theImage[y * downFactor][x * downFactor];
        ColorModelConversions::fromYCbCrToHSI(px.y, px.cb, px.cr, c[0], c[1], c[2]);
        downsized[y * w + x] = c[imgChannel - 3];
      }
    }
    break;
  }
}

static void compute_LUTs(unsigned char angle_LUT[256],
                         float sin_LUT[256 * 3],
                         float cos_LUT[256 * 3],
                         char sin_r_LUT[256 * 3][96],
                         char cos_r_LUT[256 * 3][96],
                         unsigned char angle2_LUT[256][256]) {
  for (unsigned int i = 0; i <= 255; ++i) {
    angle_LUT[i] = (unsigned char)(std::atan((float)i / 255) / 3.141592653589 * 128.);
  }
  for (unsigned int i = 0; i < 256 * 3; ++i) {
    sin_LUT[i] = (float)(std::sin((float)i * 3.141592653589 / 128.));
    cos_LUT[i] = (float)(std::cos((float)i * 3.141592653589 / 128.));
    for (unsigned char r = 0; r < 96; ++r) {
      sin_r_LUT[i][r] = (char)std::round(sin_LUT[i] * (float)r);
      cos_r_LUT[i][r] = (char)std::round(cos_LUT[i] * (float)r);
    }
  }
  for (unsigned short i = 0; i <= 255; ++i) {
    for (unsigned short j = 0; j <= 255; ++j) {
      angle2_LUT[i][j] = getBinaryAngle(angle_LUT, j - 128, i - 128);
    }
  }
}

void ShapePerceptor::grad(const unsigned char* image,
                          signed char* gradX,
                          signed char* gradY,
                          unsigned char* gradL,
                          unsigned char* gradA,
                          const unsigned int w,
                          const unsigned int h,
                          const unsigned char frame,
                          const unsigned int yStart) {

  for (unsigned int y = yStart, idx = y * w; y < h - frame; ++y) {
    for (unsigned int x = 0; x < w; x += 16, idx += 16) {
      Vec16uc a00;
      Vec16uc a02;
      Vec16uc a20;
      Vec16uc a22;
      Vec16uc a10;
      Vec16uc a12;
      Vec16uc a01;
      Vec16uc a21;
      Vec16c gx;
      Vec16c gy;
      // Do not extract outside the image edges.
      if (idx - w > 0 && idx + w < h * w - 1) {
        a00.load(image + idx - w - 1);
        a02.load(image + idx - w + 1);
        a20.load(image + idx + w - 1);
        a22.load(image + idx + w + 1);
      }
      a10.load(image + idx - 1);
      a12.load(image + idx + 1);
      gx = (-Vec16c(a00 >> 1) + Vec16c(a02 >> 1)) + (-Vec16c(a20 >> 1) + Vec16c(a22 >> 1)) + (-Vec16c(a10) + Vec16c(a12));
      a01.load(image + idx - w);
      a21.load(image + idx + w);
      gy = (-Vec16c(a00 >> 1) + Vec16c(a20 >> 1)) + (-Vec16c(a02 >> 1) + Vec16c(a22 >> 1)) + (-Vec16c(a01) + Vec16c(a21));
      gx.store(gradX + idx);
      gy.store(gradY + idx);
    }
    idx -= w;
    for (unsigned int xi = 0; xi < frame; ++xi) {
      gradX[idx + xi] = 0;
      gradY[idx + xi] = 0;
    }
    for (unsigned int xi = w - frame; xi < w; ++xi) {
      gradX[idx + xi] = 0;
      gradY[idx + xi] = 0;
    }
    idx += w;
  }
  for (unsigned int y = yStart, idx = y * w; y < h - frame; ++y) {
    for (unsigned int x = 0; x < w; x += 4, idx += 4) {
      Vec16c gx;
      Vec16c gy;
      Vec16uc gl;
      Vec4i gxi;
      Vec4i gyi;
      Vec4ui gli;
      Vec4f gxf;
      Vec4f gyf;
      Vec4f glf;
      gx.load(gradX + idx);
      gy.load(gradY + idx);
      gxi = extend_low(extend_low(gx));
      gyi = extend_low(extend_low(gy));
      gxf = to_float(gxi);
      gyf = to_float(gyi);
      glf = gxf * gxf + gyf * gyf;
      gli = round_to_int(glf * approx_rsqrt(glf)) << 1; // double the magnitude again (undo halving from downsize step)
      gl = compress(compress(gli, 0), 0);
      gl.store_partial(4, gradL + idx);
    }
  }
  const unsigned char gradThresh = (unsigned char)(peakThreshold * 255);
  for (unsigned int y = yStart, idx = y * w; y < h - frame; ++y) {
    idx += frame;
    for (unsigned int x = frame; x < w - frame; ++x, ++idx) {
      // 0 to 255 = 0deg to 360deg; East = 0deg, clockwise; direction in which gradient goes uphill
      if (gradL[idx] > gradThresh) {
        const unsigned char ga = angle2_LUT[gradY[idx] + 128][gradX[idx] + 128];
        gradA[idx] = ga;
      } else {
        gradA[idx] = 128;
      }
    }
    idx += frame;
  }
}

// Finds gradient peaks over a certain threshold & outside of BodyContour
void ShapePerceptor::peaks(const unsigned char* gradL,
                           const unsigned char* gradA,
                           unsigned char* grid,
                           std::vector<int>& list,
                           const unsigned int w,
                           const unsigned int h,
                           const unsigned int yStart,
                           const unsigned char frame) {

  const unsigned char gradThresh = (unsigned char)(peakThreshold * 255);

  for (unsigned int x = frame; x < w - frame; ++x) {

    int yEnd = (h - frame) * downFactor;
    if (clipBodyContour) {
      theBodyContour.clipBottom((int)x * downFactor, yEnd);
    }
    yEnd = std::max<int>(yEnd / downFactor, yStart);

    for (unsigned int y = yStart; y < (unsigned int)yEnd; ++y) {

      if (gradL[y * w + x] > gradThresh) {
        const unsigned char ga = gradA[y * w + x];
        const unsigned char gl = gradL[y * w + x];

        bool isPeak = false;
        if (ga < 128 + 16) {
          if (ga < 64 + 16) {
            if (ga < 32 + 16) {
              if (ga < 16) { // Right
                isPeak = (gl > gradL[(y)*w + (x - 1)]) && (gl >= gradL[(y)*w + (x + 1)]);
              } else { // Lower-right
                isPeak = (gl > gradL[(y - 1) * w + (x - 1)]) && (gl >= gradL[(y + 1) * w + (x + 1)]);
              }
            } else { // Down
              isPeak = (gl > gradL[(y - 1) * w + (x)]) && (gl >= gradL[(y + 1) * w + (x)]);
            }
          } else if (ga < 96 + 16) { // Lower-left
            isPeak = (gl > gradL[(y - 1) * w + (x + 1)]) && (gl >= gradL[(y + 1) * w + (x - 1)]);
          } else { // Left
            isPeak = (gl > gradL[(y)*w + (x + 1)]) && (gl >= gradL[(y)*w + (x - 1)]);
          }
        } else if (ga < 192 + 16) {
          if (ga < 160 + 16) { // Upper-left
            isPeak = (gl > gradL[(y + 1) * w + (x + 1)]) && (gl >= gradL[(y - 1) * w + (x - 1)]);
          } else { // Up
            isPeak = (gl > gradL[(y + 1) * w + (x)]) && (gl >= gradL[(y - 1) * w + (x)]);
          }
        } else if (ga < 224 + 16) { // Upper-right
          isPeak = (gl > gradL[(y + 1) * w + (x - 1)]) && (gl >= gradL[(y - 1) * w + (x + 1)]);
        } else { // Right
          isPeak = (gl > gradL[(y)*w + (x - 1)]) && (gl >= gradL[(y)*w + (x + 1)]);
        }

        if (!isPeak) {
          grid[y * w + x] = 0;
        } else {
          grid[y * w + x] = gl;
          list.push_back(y * w + x);
        }

      } else {
        // Below threshold
        grid[y * w + x] = 0;
      }
    }
  }
}

void ShapePerceptor::allLines(const std::vector<int>& peakList,
                              const unsigned char* gradA,
                              unsigned short* lineHough,
                              const unsigned int w,
                              const unsigned int h,
                              const unsigned int lineR,
                              const unsigned int lineRMax,
                              const unsigned int lineA,
                              unsigned char lineAOffset) {

  const unsigned char lineASpreadMax = lineAOffset; // assumes param "lineASpread" is below this; not explicitly enforced
  const signed int gradMidX = w / 2;
  const signed int gradMidY = h / 2;
  for (unsigned int i = 0, n = peakList.size(); i < n; ++i) {
    const int x = peakList[i] % w;
    const int y = peakList[i] / w;

    const unsigned char a0 = gradA[y * w + x];
    unsigned char votes = 1;
    if (voteUsingRatios) {
      votes = (unsigned char)std::round(std::abs(sin_LUT[a0 + 256]) +
                                        std::abs(cos_LUT[a0 + 256]) * ratio_LUT[y / ratio_LUT_interval]);
    }

    const signed int aMin = a0 - lineASpread;
    const signed int aMax = a0 + lineASpread;
    for (signed int a = aMin; a <= aMax; ++a) {
      const signed int r =
        (signed int)std::round((x - gradMidX) * cos_LUT[a + 256] + (y - gradMidY) * sin_LUT[a + 256]) + lineRMax;
      if (r >= 0 && r < (signed int)lineR) {
        lineHough[(a + lineAOffset) * lineR + r] += votes;
      }
    }
  }
  // Wrap around a < 0 and a > 255
  // TODO This is retarded, don't lay-out lineHough so that this wrap around trick is needed,
  //      it is extremely annoying to deal with afterwards
  for (unsigned int a = 0; a < lineASpreadMax; ++a) {
    for (unsigned int r = 0; r < lineR; ++r) {
      lineHough[(a + 256) * lineR + r] += lineHough[a * lineR + r];
      lineHough[a * lineR + r] = 0;
    }
  }
  for (unsigned int a = lineA - lineASpreadMax; a < lineA; ++a) {
    for (unsigned int r = 0; r < lineR; ++r) {
      lineHough[(a - 256) * lineR + r] += lineHough[a * lineR + r];
      lineHough[a * lineR + r] = 0;
    }
  }
}

void ShapePerceptor::findLinePeaks(const unsigned short* lineHough,
                                   unsigned short* linePeaks,
                                   std::vector<unsigned int>& linePeakList,
                                   const unsigned int lineR,
                                   const unsigned int lineA,
                                   const unsigned char lineAOffset) {

  unsigned short lineHoughMax = 0;

  // Find all local maxima with high votes in lineHough => linePeaks, linePeakList
  for (unsigned int a = lineAOffset, idx = a * lineR; a < lineA - lineAOffset; ++a) {
    for (unsigned int r = 0; r < lineR; ++r, ++idx) {
      if (lineHough[idx] >= lineMinVotes && lineHough[idx] > lineHough[idx - lineR - 1] &&
          lineHough[idx] > lineHough[idx - lineR] && lineHough[idx] > lineHough[idx - lineR + 1] &&
          lineHough[idx] > lineHough[idx - 1] && lineHough[idx] > lineHough[idx + 1] &&
          lineHough[idx] > lineHough[idx + lineR - 1] && lineHough[idx] > lineHough[idx + lineR] &&
          lineHough[idx] > lineHough[idx + lineR + 1]) {
        linePeaks[idx] = lineHough[idx];
        linePeakList.push_back(idx);
        if (lineHough[idx] >= lineHoughMax) {
          lineHoughMax = lineHough[idx];
        }
      }
    }
  }
}

// TODO Calculate aSpread & rSpread from real-world sizes
void ShapePerceptor::excludeSimilarLines(const unsigned short* linePeaks,
                                         const std::vector<unsigned int>& linePeakList,
                                         unsigned short* filteredPeaks,
                                         std::vector<unsigned int>& filteredPeakList,
                                         const unsigned int lineR) {

  // For each peak, if local max within (+/- lineASpread, +/- lineRSpread), add to filteredPeakList
  for (unsigned int i = 0; i < linePeakList.size(); ++i) {
    const unsigned int a =
      (linePeakList[i] / lineR); // "a" in memory layout ("a = x" means angle is actually "x - lineAOffset")
    const unsigned int r = (linePeakList[i] % lineR);
    const unsigned short value = linePeaks[a * lineR + r];
    bool valueIsMaggs = true;
    for (unsigned int a2 = a - lineASpread; a2 <= a + lineASpread; ++a2) {
      for (unsigned int r2 = r - lineRSpread; r2 <= r + lineRSpread; ++r2) {
        if (linePeaks[a2 * lineR + r2] > value) {
          valueIsMaggs = false;
          break;
        }
      }
      if (!valueIsMaggs) {
        break;
      }
    }
    if (valueIsMaggs) {
      filteredPeakList.push_back(a * lineR + r);
      filteredPeaks[a * lineR + r] = linePeaks[a * lineR + r];
    }
  }
}

// Turn lines into segments
void ShapePerceptor::segmentLines(const unsigned short* lineVotes,
                                  const std::vector<unsigned int>& lineList,
                                  const unsigned char* gradA,
                                  const unsigned char* edges,
                                  std::vector<ShapePercept::LineSegment>& segments,
                                  unsigned char* linesErased,
                                  const unsigned int w,
                                  const unsigned int h,
                                  const unsigned int lineR,
                                  const unsigned int lineRMax,
                                  const unsigned int lineA,
                                  const unsigned char lineAOffset) {

  const unsigned char gradThresh = (unsigned char)(peakThreshold * 255);

  memcpy(linesErased, edges, 320 * 240 * sizeof(unsigned char));

  for (unsigned int i = 0; i < std::min<unsigned short>(lineList.size(), maxLines); ++i) {
    const unsigned char a = (unsigned char)((int)(lineList[i] / lineR) - (int)lineAOffset);
    const float r = (int)(lineList[i] % lineR) - (int)lineRMax;
    const float cosA = cos_LUT[a + 256];
    const float sinA = sin_LUT[a + 256];

    const signed int xTop = (signed int)std::round((r - (float)((float)1 - (float)h / 2) * sinA) / cosA + (float)w / 2);
    const signed int xBottom = (signed int)std::round((r - (float)((float)-1 + (float)h / 2) * sinA) / cosA + (float)w / 2);
    const signed int yLeft = (signed int)std::round((r - (float)((float)1 - (float)w / 2) * cosA) / sinA + (float)h / 2);
    const signed int yRight = (signed int)std::round((r - (float)((float)-1 + (float)w / 2) * cosA) / sinA + (float)h / 2);

    // TODO Use pixel ratios to increment hits?
    unsigned char hits;   // Consecutive hits
    unsigned char misses; // Consecutive misses
    Vector2<int> p1(-1, -1);
    Vector2<int> p2(-1, -1);
    const Vector2<int> emptyPoint(-1, -1);
    ShapePercept::LineSegment segment;
    bool inSegment = false;

    if ((sinA >= 0 ? sinA : -sinA) < std::sqrt(0.5)) { // Near vertical
      const signed int yLow = (a == 0 || a == 128) ? 1 : std::max<signed int>(1, std::min<signed int>(yLeft, yRight));
      const signed int yHigh =
        (a == 0 || a == 128) ? h - 1 : std::min<signed int>(h - 1, std::max<signed int>(yLeft, yRight));

      hits = 0;
      misses = 0;
      for (signed int y = yLow; y <= yHigh; ++y) { // Scan from top to bottom
        const signed int x = (signed int)std::round((r - (float)((float)y - (float)h / 2) * sinA) / cosA + (float)w / 2);
        if (x >= 1 && x < (signed int)w - 1) { // Offset 1 to prevent segfault
          const bool hit =
            (linesErased[(y)*w + (x - 1)] > gradThresh && angleDifference(gradA[(y)*w + (x - 1)], a) <= lineASpread) ||
            (linesErased[(y)*w + (x)] > gradThresh && angleDifference(gradA[(y)*w + (x)], a) <= lineASpread) ||
            (linesErased[(y)*w + (x + 1)] > gradThresh && angleDifference(gradA[(y)*w + (x + 1)], a) <= lineASpread);

          if (newSegmenter) {

            if (hit) {
              if (p1 == emptyPoint) {
                p1.x = x;
                p1.y = y;
              } else {
                p2.x = x;
                p2.y = y;
              }
              ++hits;
              misses = 0;
            } else { // miss
              ++misses;
              if (misses >= lineMaxMisses) {
                if (hits >= lineMinHits) {
                  if (p1 == p2) {
                    OUTPUT(idText, text, "p1 == p2");
                  }
                  if (p1 == emptyPoint) {
                    OUTPUT(idText, text, "p1 == emptyPoint");
                  }
                  if (p2 == emptyPoint) {
                    OUTPUT(idText, text, "p2 == emptyPoint");
                  }

                  segment.p1 = p1;
                  segment.p2 = p2;
                  segment.votes = lineVotes[lineList[i]];
                  segment.hits = hits;

                  segments.push_back(segment);
                  eraseNearSegment(segment, linesErased, w, h);
                }

                // Reset things
                p1 = emptyPoint;
                p2 = emptyPoint;
                hits = 0;
                misses = 0;
              }
            }

          } else { // old segmenter

            if (!inSegment) {
              if (hit) {
                ++hits;
                if (hits >= oldLineMinHits) {
                  p1.y = y - oldLineMinHits;
                  p1.x = (signed int)std::round((r - (float)((float)p1.y - (float)h / 2) * sinA) / cosA + (float)w / 2);
                  inSegment = true;
                  misses = 0;
                }
              } else {
                hits = 0;
              }
            } else { // inSegment
              if (!hit) {
                ++misses;
                if (misses >= oldLineMinMisses) {
                  p2.y = y - oldLineMinMisses;
                  p2.x = (signed int)std::round((r - (float)((float)p2.y - (float)h / 2) * sinA) / cosA + (float)w / 2);

                  segment.p1 = p1;
                  segment.p2 = p2;
                  segment.votes = lineVotes[lineList[i]];
                  segment.hits = hits;

                  if (p1 != p2) { // Avoid 1px "point segments"
                    segments.push_back(segment);
                    eraseNearSegment(segment, linesErased, w, h);
                  }

                  inSegment = false;
                  hits = 0;
                }
              } else { // Miss
                misses = 0;
              }
            }
          }
        }
      }

      if (newSegmenter) {

        if (hits >= lineMinHits) { // Hit edge of image
          if (p1 == p2) {
            OUTPUT(idText, text, "p1 == p2");
          }
          if (p1 == emptyPoint) {
            OUTPUT(idText, text, "p1 == emptyPoint");
          }
          if (p2 == emptyPoint) {
            OUTPUT(idText, text, "p2 == emptyPoint");
          }

          segment.p1 = p1;
          segment.p2 = p2;
          segment.votes = lineVotes[lineList[i]];
          segment.hits = hits;

          segments.push_back(segment);
          eraseNearSegment(segment, linesErased, w, h);
        }

        // Reset things
        p1 = emptyPoint;
        p2 = emptyPoint;
        hits = 0;
        misses = 0;

      } else { // old segmenter

        if (inSegment && p1 != Vector2<int>(-1, -1)) { // Hit edge of image while inSegment
          p2.y = yHigh - misses;
          p2.x = (signed int)std::round((r - (float)((float)p2.y - (float)h / 2) * sinA) / cosA + (float)w / 2);

          segment.p1 = p1;
          segment.p2 = p2;
          segment.votes = lineVotes[lineList[i]];
          segment.hits = hits;

          if (p1 != p2) { // Avoid 1px "point segments"
            segments.push_back(segment);
            eraseNearSegment(segment, linesErased, w, h);
          }

          inSegment = false;
          hits = 0;
        }
      }

    } else { // Near horizontal
      const signed int xLow = (a == 64 || a == 192) ? 1 : std::max<signed int>(1, std::min<signed int>(xTop, xBottom));
      const signed int xHigh =
        (a == 64 || a == 192) ? w - 1 : std::min<signed int>(w - 1, std::max<signed int>(xTop, xBottom));

      hits = 0;
      misses = 0;
      for (signed int x = xLow; x <= xHigh; ++x) { // Scan from left to right
        const signed int y = (signed int)std::round((r - (float)((float)x - (float)w / 2) * cosA) / sinA + (float)h / 2);
        if (y >= 1 && y < (signed int)h - 1) { // Offset 1 to prevent segfault
          const bool hit =
            (linesErased[(y - 1) * w + (x)] > gradThresh && angleDifference(gradA[(y - 1) * w + (x)], a) <= lineASpread) ||
            (linesErased[(y)*w + (x)] > gradThresh && angleDifference(gradA[(y)*w + (x)], a) <= lineASpread) ||
            (linesErased[(y + 1) * w + (x)] > gradThresh && angleDifference(gradA[(y + 1) * w + (x)], a) <= lineASpread);

          if (newSegmenter) {

            if (hit) {
              if (p1 == emptyPoint) {
                p1.x = x;
                p1.y = y;
              } else {
                p2.x = x;
                p2.y = y;
              }
              ++hits;
              misses = 0;
            } else { // miss
              ++misses;
              if (misses >= lineMaxMisses) {
                if (hits >= lineMinHits) {
                  if (p1 == p2) {
                    OUTPUT(idText, text, "p1 == p2");
                  }
                  if (p1 == emptyPoint) {
                    OUTPUT(idText, text, "p1 == emptyPoint");
                  }
                  if (p2 == emptyPoint) {
                    OUTPUT(idText, text, "p2 == emptyPoint");
                  }

                  segment.p1 = p1;
                  segment.p2 = p2;
                  segment.votes = lineVotes[lineList[i]];
                  segment.hits = hits;

                  segments.push_back(segment);
                  eraseNearSegment(segment, linesErased, w, h);
                }

                // Reset things
                p1 = emptyPoint;
                p2 = emptyPoint;
                hits = 0;
                misses = 0;
              }
            }

          } else { // old segmenter

            if (!inSegment) {
              if (hit) {
                ++hits;
                if (hits >= oldLineMinHits) {
                  p1.x = x - oldLineMinHits;
                  p1.y = (signed int)std::round((r - (float)((float)p1.x - (float)w / 2) * cosA) / sinA + (float)h / 2);
                  inSegment = true;
                  misses = 0;
                }
              } else { // Miss when out of segment
                hits = 0;
              }
            } else { // inSegment
              if (!hit) {
                ++misses;
                if (misses >= oldLineMinMisses) {
                  p2.x = x - oldLineMinMisses;
                  p2.y = (signed int)std::round((r - (float)((float)p2.x - (float)w / 2) * cosA) / sinA + (float)h / 2);

                  segment.p1 = p1;
                  segment.p2 = p2;
                  segment.votes = lineVotes[lineList[i]];
                  segment.hits = hits;

                  if (p1 != p2) { // Avoid 1px "point segments"
                    segments.push_back(segment);
                    eraseNearSegment(segment, linesErased, w, h);
                  }

                  inSegment = false;
                  hits = 0;
                }
              } else { // Hit when in segment
                misses = 0;
              }
            }
          }
        }
      }

      if (newSegmenter) {

        if (hits >= lineMinHits) { // Hit edge of image
          if (p1 == p2) {
            OUTPUT(idText, text, "p1 == p2");
          }
          if (p1 == emptyPoint) {
            OUTPUT(idText, text, "p1 == emptyPoint");
          }
          if (p2 == emptyPoint) {
            OUTPUT(idText, text, "p2 == emptyPoint");
          }

          segment.p1 = p1;
          segment.p2 = p2;
          segment.votes = lineVotes[lineList[i]];
          segment.hits = hits;

          segments.push_back(segment);
          eraseNearSegment(segment, linesErased, w, h);
        }

        // Reset things
        p1 = emptyPoint;
        p2 = emptyPoint;
        hits = 0;
        misses = 0;

      } else { // old segmenter

        if (inSegment && p1 != Vector2<int>(-1, -1)) { // Hit edge of image while inSegment
          p2.x = xHigh - misses;
          p2.y = (signed int)std::round((r - (float)((float)p2.x - (float)w / 2) * cosA) / sinA + (float)h / 2);

          segment.p1 = p1;
          segment.p2 = p2;
          segment.votes = lineVotes[lineList[i]];
          segment.hits = hits;

          if (p1 != p2) { // Avoid 1px "point segments"
            segments.push_back(segment);
            eraseNearSegment(segment, linesErased, w, h);
          }

          inSegment = false;
          hits = 0;
        }
      }
    }
  }
}

// Erase one segment from a grid
void ShapePerceptor::eraseNearSegment(const ShapePercept::LineSegment& segment,
                                      unsigned char* grid,
                                      const unsigned int w,
                                      const unsigned int h) {

  const Vector2<int> p1 = segment.p1;
  const Vector2<int> p2 = segment.p2;

  const int dx = p2.x - p1.x;
  const int dy = p2.y - p1.y;

  if (std::abs(dy) > std::abs(dx)) { // Near vertical
    for (signed int y = p1.y; y <= p2.y; ++y) {
      const signed int x = (signed int)std::round((float)(y - p1.y) * (float)dx / (float)dy + p1.x);
      /*if (x >= 2 && x < (signed int)w-2) {  // Offset 2 to prevent segfault; could be done better but whatever
        grid[(y)*w+(x-2)] = 0;
        grid[(y)*w+(x-1)] = 0;
        grid[(y)*w+(x)] = 0;
        grid[(y)*w+(x+1)] = 0;
        grid[(y)*w+(x+2)] = 0;
      } else */
      if (x >= 1 && x < (signed int)w - 1) {
        grid[(y)*w + (x - 1)] = 0;
        grid[(y)*w + (x)] = 0;
        grid[(y)*w + (x + 1)] = 0;
      }
    }

  } else { // Near horizontal
    for (signed int x = p1.x; x <= p2.x; ++x) {
      const signed int y = (signed int)std::round((float)(x - p1.x) * (float)dy / (float)dx + p1.y);
      /*if (y >= 2 && y < (signed int)h-2) {  // Offset 2 to prevent segfault; could be done better but whatever
        grid[(y-2)*w+(x)] = 0;
        grid[(y-1)*w+(x)] = 0;
        grid[(y)*w+(x)] = 0;
        grid[(y+1)*w+(x)] = 0;
        grid[(y+2)*w+(x)] = 0;
      } else */
      if (y >= 1 && y < (signed int)h - 1) {
        grid[(y - 1) * w + (x)] = 0;
        grid[(y)*w + (x)] = 0;
        grid[(y + 1) * w + (x)] = 0;
      }
    }
  }
}

void ShapePerceptor::listNonzero(const unsigned char* grid,
                                 std::vector<int>& list,
                                 const unsigned int w,
                                 const unsigned int h,
                                 const unsigned int yStart,
                                 const unsigned char frame) {

  for (signed int y = yStart; y < (signed int)(h - frame); ++y) {
    for (signed int x = frame; x < (signed int)(w - frame); ++x) {
      if (grid[y * w + x] > 0) {
        list.push_back(y * w + x);
      }
    }
  }
}

void ShapePerceptor::allCircles(const std::vector<int>& peakList,
                                const unsigned char* gradA,
                                unsigned char* circleHough,
                                const unsigned int w,
                                const unsigned int h) {

  int prev_y = -1;
  int r = 0;
  int n = peakList.size();
  int nIter = std::min<int>(n, circMaxIter);
  bool randomMode = ((float)nIter / n) < 0.7f;
  for (int s = nIter - 1; s >= 0; --s) {
    int i = randomMode ? (rand() % n) : (s + n - nIter);

    const int x = peakList[i] % w;
    const int y = peakList[i] / w;

    if (y != prev_y) {
      if (!ballRadiusInImage(y, downFactor, r)) {
        continue;
      }
    }

    if (r <= (signed char)ballMinR) {
      continue; // Unlikely we'll ever find a ball this small
    }

    // IDEA Smaller radii cast less votes, but with more weight; larger radii cast more votes, but with less weight;
    // sum of votes cast by one point is a fixed 100
    const unsigned char a = gradA[y * w + x];
    for (char dr = -circRSpread; dr < circRSpread; ++dr) {
      if ((signed)r + dr <= ballMinR) {
        continue;
      }

      const unsigned char r2 = (unsigned char)r + dr;

      // "Forward" side
      for (char dth = -circASpread; dth < circASpread; ++dth) {
        const int x2 = x + (int)cos_r_LUT[a + dth + 256][r2];
        const int y2 = y + (int)sin_r_LUT[a + dth + 256][r2];
        if (x2 >= 0 && y2 >= 0 && x2 < (int)w && y2 < (int)h) {
          ++circleHough[y2 * w + x2];
        }
      }
      // "Backward" side
      for (char dth = -circASpread; dth < circASpread; ++dth) {
        const int x2 = x + (int)cos_r_LUT[a + dth + 128][r2];
        const int y2 = y + (int)sin_r_LUT[a + dth + 128][r2];
        if (x2 >= 0 && y2 >= 0 && x2 < (int)w && y2 < (int)h) {
          ++circleHough[y2 * w + x2];
        }
      }
    }
  }
}

void ShapePerceptor::findBalls(const std::vector<int>& edgeList,
                               const unsigned char* circleHough,
                               std::vector<std::pair<unsigned int, unsigned char>>& circleList,
                               unsigned char* circlePeaks,
                               const unsigned int w,
                               const unsigned int h,
                               const unsigned int yStart,
                               const unsigned char frame) {

  int nIter = std::min<int>(edgeList.size() + 1, circMaxIter);
  int scaledMinVotes = circMinVotes * (int)((float)nIter / (1 + edgeList.size()));
  const unsigned int yBottom = h - frame - 1;

  for (unsigned int x = frame + 1; x < w - frame; ++x) {
    unsigned int yTop = yStart + 1;

    if (theCameraInfo.camera == CameraInfo::upper) {
      if (theFieldBoundary.model.size() == 4) {
        unsigned int y_border =
          (unsigned int)Geometry::findBoundaryY((float)((float)(x)*downFactor * 1.0f), theFieldBoundary, theCameraInfo);
        if (y_border > yTop * downFactor) {
          yTop = (unsigned int)(y_border / downFactor * 1.0f);
        }
      }
    }
    for (unsigned int y = yBottom - 1; y >= yTop; --y) {
      if (circleHough[y * w + x] > circleHough[(y - 2) * w + (x - 2)] &&
          circleHough[y * w + x] > circleHough[(y - 2) * w + (x - 1)] &&
          circleHough[y * w + x] > circleHough[(y - 2) * w + (x)] &&
          circleHough[y * w + x] > circleHough[(y - 2) * w + (x + 1)] &&
          circleHough[y * w + x] > circleHough[(y - 2) * w + (x + 2)] &&
          circleHough[y * w + x] > circleHough[(y - 1) * w + (x - 2)] &&
          circleHough[y * w + x] > circleHough[(y - 1) * w + (x - 1)] &&
          circleHough[y * w + x] > circleHough[(y - 1) * w + (x)] &&
          circleHough[y * w + x] > circleHough[(y - 1) * w + (x + 1)] &&
          circleHough[y * w + x] > circleHough[(y - 1) * w + (x + 2)] &&
          circleHough[y * w + x] > circleHough[(y)*w + (x - 2)] && circleHough[y * w + x] > circleHough[(y)*w + (x - 1)] &&
          circleHough[y * w + x] > circleHough[(y)*w + (x + 1)] && circleHough[y * w + x] > circleHough[(y)*w + (x + 2)] &&
          circleHough[y * w + x] > circleHough[(y + 1) * w + (x - 2)] &&
          circleHough[y * w + x] > circleHough[(y + 1) * w + (x - 1)] &&
          circleHough[y * w + x] > circleHough[(y + 1) * w + (x)] &&
          circleHough[y * w + x] > circleHough[(y + 1) * w + (x + 1)] &&
          circleHough[y * w + x] > circleHough[(y + 1) * w + (x + 2)] &&
          circleHough[y * w + x] > circleHough[(y + 2) * w + (x - 2)] &&
          circleHough[y * w + x] > circleHough[(y + 2) * w + (x - 1)] &&
          circleHough[y * w + x] > circleHough[(y + 2) * w + (x)] &&
          circleHough[y * w + x] > circleHough[(y + 2) * w + (x + 1)] &&
          circleHough[y * w + x] > circleHough[(y + 2) * w + (x + 2)]) {
        int gaussianFilteredValue = 4 * circleHough[y * w + x];
        gaussianFilteredValue += circleHough[(y - 1) * w + (x - 1)];
        gaussianFilteredValue += 2 * circleHough[(y - 1) * w + x];
        gaussianFilteredValue += circleHough[(y - 1) * w + (x + 1)];
        gaussianFilteredValue += circleHough[(y + 1) * w + (x - 1)];
        gaussianFilteredValue += 2 * circleHough[(y + 1) * w + x];
        gaussianFilteredValue += circleHough[(y + 1) * w + (x + 1)];
        gaussianFilteredValue += 2 * circleHough[y * w + (x - 1)];
        gaussianFilteredValue += 2 * circleHough[y * w + (x + 1)];
        bool consider_region = circleHough[y * w + x] > scaledMinVotes;

        if (theCameraInfo.camera == CameraInfo::upper) {
          // TODO Include logic for checking whether the ball can be inside the region or not
          // Incase the region is inside a robot or field, shift the region

          for (const auto& player : thePlayerPercept.players) {
            int offX = ballPlayerAllowedPercentage * (player.x2 - player.x1) / 2;
            int offY = ballPlayerAllowedPercentage * (player.y2 - player.y1) / 2;
            if (player.x1 + offX < x * downFactor && downFactor * x < player.x2 - offX &&
                player.y1 + offY < downFactor * y && downFactor * y < player.y2 - offY) {
              consider_region = false;
              break;
            }
          }
        }
        if (consider_region) {
          circleList.push_back(std::pair<unsigned int, unsigned char>(y * w + x, circleHough[y * w + x]));
          circlePeaks[y * w + x] = circleHough[y * w + x];
        }
      }
    }
  }

  // Keep n highest
  std::sort(circleList.begin(),
            circleList.end(),
            [](const std::pair<unsigned int, unsigned char>& a, const std::pair<unsigned int, unsigned char>& b) {
              return a.second > b.second;
            });
  for (unsigned int i = 0; i < std::min<unsigned int>(ballCandidates, circleList.size()); ++i) {
    const unsigned int ballPos = circleList[i].first;
    ballList.push_back(ballPos);
  }
}

// Use thresholding on edges & downsized image to find edges of bright white objects
void ShapePerceptor::findBrightEdges(const unsigned char* downsized,
                                     const unsigned char* edges,
                                     const unsigned char* gradA,
                                     unsigned char* brightEdges,
                                     unsigned char whiteThresh,
                                     unsigned char edgeThresh,
                                     unsigned int w,
                                     unsigned int h,
                                     unsigned int yStart,
                                     unsigned char frame) {

  ASSERT(frame >= 1);

  for (unsigned int y = yStart; y < h - frame; ++y) {
    for (unsigned int x = frame; x < w - frame; ++x) {
      if (edges[y * w + x] > edgeThresh) {
        const unsigned char ga = gradA[y * w + x];

        // No segfaults as long as frame >= 1
        bool isWhite = false;
        if (ga < 128 + 16) {
          if (ga < 64 + 16) {
            if (ga < 32 + 16) {
              if (ga < 16) { // Right
                isWhite = (downsized[(y)*w + (x + 1)] > whiteThresh);
              } else { // Lower-right
                isWhite = (downsized[(y + 1) * w + (x + 1)] > whiteThresh);
              }
            } else { // Down
              isWhite = (downsized[(y + 1) * w + (x)] > whiteThresh);
            }
          } else if (ga < 96 + 16) { // Lower-left
            isWhite = (downsized[(y + 1) * w + (x - 1)] > whiteThresh);
          } else { // Left
            isWhite = (downsized[(y)*w + (x - 1)] > whiteThresh);
          }
        } else if (ga < 192 + 16) {
          if (ga < 160 + 16) { // Upper-left
            isWhite = (downsized[(y - 1) * w + (x - 1)] > whiteThresh);
          } else { // Up
            isWhite = (downsized[(y - 1) * w + (x)] > whiteThresh);
          }
        } else if (ga < 224 + 16) { // Upper-right
          isWhite = (downsized[(y - 1) * w + (x + 1)] > whiteThresh);
        } else { // Right
          isWhite = (downsized[(y)*w + (x + 1)] > whiteThresh);
        }

        if (isWhite) {
          brightEdges[y * w + x] = edges[y * w + x];
        } else {
          brightEdges[y * w + x] = 0;
        }
      }
    }
  }
}

// Erases inside edgesNoCircles according to the circles in circleList
void ShapePerceptor::eraseInsideCircles(const std::vector<std::pair<unsigned int, unsigned char>>& circleList,
                                        const unsigned char* brightEdges,
                                        unsigned char* edgesNoCircles,
                                        unsigned int w,
                                        unsigned int h,
                                        unsigned char frame) {

  memcpy(edgesNoCircles, brightEdges, w * h * sizeof(unsigned char));

  for (unsigned int i = 0; i < circleList.size(); ++i) {
    const int x = circleList[i].first % w;
    const int y = circleList[i].first / w;

    int r;
    if (!ballRadiusInImage(Vector2<int>(x, y), downFactor, r)) {
      continue;
    }
    r += 1; // 1 as an extra margin

    for (int y2 = -r; y2 <= r; ++y2) {
      for (int x2 = -r; x2 <= r; ++x2) {
        if (y2 * y2 + x2 * x2 <= r * r && (x + x2) >= 0 && (x + x2) < (int)w && (y + y2) >= 0 && (y + y2) < (int)h) {
          edgesNoCircles[(y + y2) * w + (x + x2)] = 0;
        }
      }
    }
  }
}

void ShapePerceptor::findPenaltyMark(const unsigned char* edgesNoCircles,
                                     const unsigned char* edges,
                                     bool& seen,
                                     Vector2<int>& position,
                                     unsigned int w,
                                     unsigned int h,
                                     int yTop,
                                     unsigned char frame,
                                     int size) {

  int eps = 5; // Size of penalty mark within +-eps (Camera frame)
  int yMax = 0;
  int yMin = 320;
  int xMax = 0;
  int xMin = 0;
  bool success = false;
  bool notPM = false;
  unsigned char edgesNoCirclesErased[320 * 240] = {0};
  int yLowest = int(h - frame);
  int pixelThresh = 10;
  int counter = 0;
  // unsigned char* edgesNoCirclesErased = edgesNoCircles;

  Vector2<> pointOnField(-1, -1);
  Vector2<> pointOnFieldEndX(-1, -1);
  Vector2<> pointOnFieldEndY(-1, -1);
  Vector2<int> pointInImageEndNew(-1, -1);
  Vector2<int> pointInImageEndX(-1, -1);
  Vector2<int> pointInImageEndY(-1, -1);

  // Start from bottom
  for (int y = h - frame; y > yTop; --y) {
    if (success) {
      break;
    }
    notPM = false;
    // look if there is something
    for (int x = frame; x < w - frame; ++x) {
      if (success) {
        break;
      }
      if (edgesNoCircles[y * w + x] != 0 && edgesNoCirclesErased[y * w + x] != 1) {
        // Erase points that have already been checked
        edgesNoCirclesErased[y * w + x] = 1;
        // Calculate how big the penalty mark should be in the Camera frame at that distance
        Geometry::calculatePointOnField(x * downFactor, y * downFactor, theCameraMatrix, theCameraInfo, pointOnField);
        pointOnFieldEndX.x = pointOnField.x + size * cosf(theCameraMatrix.rotation.getZAngle());
        pointOnFieldEndX.y = pointOnField.y + size * sinf(theCameraMatrix.rotation.getZAngle());
        pointOnFieldEndY.y = pointOnField.y - size * cosf(theCameraMatrix.rotation.getZAngle());
        pointOnFieldEndY.x = pointOnField.x + size * sinf(theCameraMatrix.rotation.getZAngle());

        if (Geometry::calculatePointInImage(pointOnFieldEndX, theCameraMatrix, theCameraInfo, pointInImageEndY) &&
            Geometry::calculatePointInImage(pointOnFieldEndY, theCameraMatrix, theCameraInfo, pointInImageEndX)) {
          if (pointInImageEndX.x / downFactor < (int)(w - frame) && pointInImageEndX.y / downFactor > yTop &&
              pointInImageEndX.x / downFactor > frame && pointInImageEndX.y / downFactor < (int)(h - frame)) {
            if (pointInImageEndY.x / downFactor < (int)(w - frame) && pointInImageEndY.y / downFactor > yTop &&
                pointInImageEndY.x / downFactor > frame && pointInImageEndY.y / downFactor < (int)(h - frame)) {
              pointInImageEndNew.x = pointInImageEndX.x / downFactor;
              pointInImageEndNew.y = pointInImageEndY.y / downFactor;

              yMax = y;
              xMin = x;
              yMin = 320;
              xMax = 0;
              // check area x-100 to x+100 and y to y-100, if candidate is within penalty mark size

              for (int y2 = y; y2 > std::max(pointInImageEndNew.y - eps, 0); --y2) {
                if (y2 < 0 || y2 >= h) {
                  OUTPUT_WARNING("ShapePerceptor: y2 = " << y2 << ", y = " << y << ", pointInImageEndNew = ("
                                                         << pointInImageEndNew.x << ", " << pointInImageEndNew.y << ")");
                }
                if (notPM) {
                  break;
                }
                for (int x2 = std::max(x - (pointInImageEndNew.x - x), 0); x2 < pointInImageEndNew.x; ++x2) {
                  if (x2 < 0 || x2 >= w) {
                    OUTPUT_WARNING("ShapePerceptor: x2 = " << x2 << ", x = " << x << ", pointInImageEndNew = ("
                                                           << pointInImageEndNew.x << ", " << pointInImageEndNew.y << ")");
                  }
                  if (edgesNoCircles[y2 * w + x2] != 0) {
                    edgesNoCirclesErased[y2 * w + x2] = 1;
                    if (y2 < yMin) {
                      yMin = y2;
                    }
                    if (x2 > xMax) {
                      xMax = x2;
                    } else if (x2 < xMin) {
                      xMin = x2;
                    }
                    if (xMax - xMin > (pointInImageEndNew.x - xMin) + eps) {
                      notPM = true;
                      // OUTPUT_TEXT("Break 1");
                      break;
                    }
                  }
                }
              }
              if (!notPM && ((yMax - yMin) > (yMax - pointInImageEndNew.y) / sqrt(2)) &&
                  ((xMax - xMin) > (pointInImageEndNew.x - xMin) / sqrt(2))) {
                counter = 0;
                // check if surrounded by nothing
                for (int yN = std::min(yMax + 2 * (yMax - pointInImageEndNew.y), yLowest);
                     yN > yMin - 2 * (yMax - pointInImageEndNew.y);
                     --yN) {
                  if (yN < yTop) {
                    break;
                  }
                  if (yN < 0 || yN >= h) {
                    OUTPUT_WARNING("ShapePerceptor: yN = " << yN << ", yMin = " << yMin << ", yMax = " << yMax
                                                           << ", y = " << y << ", pointInImageEnd = ("
                                                           << pointInImageEndNew.x << ", " << pointInImageEndNew.y << ")");
                  }
                  for (int xN = std::max(xMin - 2 * (pointInImageEndNew.x - xMin), 0);
                       xN < xMax + 2 * (pointInImageEndNew.x - xMin);
                       ++xN) {
                    if (xN > w - frame) {
                      break;
                    }
                    if (xN < 0 || xN >= w) {
                      OUTPUT_WARNING("ShapePerceptor: xN = " << xN << ", xMin = " << xMin << ", xMax = " << xMax
                                                             << ", x = " << x << ", pointInImageEnd = ("
                                                             << pointInImageEndNew.x << ", " << pointInImageEndNew.y << ")");
                    }
                    if (edges[yN * w + xN] != 0 && (xN < xMin || xN > xMax || yN > yMax || yN < yMin)) {
                      if (counter > pixelThresh) {
                        notPM = true;
                        // OUTPUT_TEXT("Break 2");
                        break;
                      } else {
                        counter = counter + 1;
                      }
                    }
                  }
                }
                if (!notPM) {
                  success = true;
                  // OUTPUT_TEXT("Penalty mark found! ");
                }
              }
            }
          }
        }
      }
    }
  }
  seen = success;
  position.y = yMin + (yMax - yMin) / 2;
  position.x = xMin + (xMax - xMin) / 2;

  if (success) {
    LINE("module:ShapePerceptor:lines", xMin, 0, xMin, h, 1, Drawings::ps_solid, ColorClasses::blue);
    LINE("module:ShapePerceptor:lines", xMax, 0, xMax, h, 1, Drawings::ps_solid, ColorClasses::blue);
    LINE("module:ShapePerceptor:lines", 0, yMin, w, yMin, 1, Drawings::ps_solid, ColorClasses::blue);
    LINE("module:ShapePerceptor:lines", 0, yMax, w, yMax, 1, Drawings::ps_solid, ColorClasses::blue);
  }
}

void ShapePerceptor::drawTheImage(const void* grid,
                                  const int pxType,
                                  const unsigned int drawW,
                                  const unsigned int drawH,
                                  const unsigned int w,
                                  const unsigned int h,
                                  const unsigned int yFar,
                                  const bool isOptical,
                                  const unsigned char bias) {

  unsigned char c[3]; // y, cb, cr
  unsigned char s[1]; // y
  COMPLEX_DEBUG_IMAGE(shapes, {
    if (drawImage) {
      INIT_DEBUG_IMAGE_BLACK(shapes, drawW, drawH);

      c[0] = bias;
      c[1] = c[2] = 0;
      s[0] = 1;
      if (!grid) {
        for (unsigned int y = 0; y < drawH; ++y) {
          if (isOptical && y == yFar * drawW / w / downFactor)
            c[2] = 48;
          else
            c[1] = c[2] = 0;

          for (unsigned int x = 0; x < drawW; ++x) {
            DEBUG_IMAGE_SET_PIXEL_YUV(
              shapes, x, y, theImage[y][x].y + c[0], theImage[y][x].cb + c[1], theImage[y][x].cr + c[2]);
          }
        }
      } else {
        for (unsigned int y = 0; y < drawH; ++y) {
          if (isOptical && y == yFar * drawW / w / downFactor)
            c[2] = 48;
          else if (!isOptical || drawWhich == 11 || drawWhich == 12)
            c[2] = (unsigned char)-128;
          else
            c[1] = c[2] = 0;

          if (drawWhich == 11 || drawWhich == 12) // downsized(Y), circleHough, circlePeaks
            s[0] = 2;
          else if (drawWhich == 1 && (imgChannel == 1))
            s[0] = 4;

          if (pxType == 1) {
            for (unsigned int x = 0; x < drawW; ++x) {
              DEBUG_IMAGE_SET_PIXEL_YUV(shapes,
                                        x,
                                        y,
                                        (unsigned char)((int)(((unsigned char*)grid)[y * drawW + x]) * s[0] + c[0]),
                                        128 + c[1],
                                        128 + c[2]);
            }
          } else if (pxType == 2) {
            const int darkenFactor = 2;
            for (unsigned int x = 0; x < drawW; ++x) {
              DEBUG_IMAGE_SET_PIXEL_YUV(
                shapes,
                x,
                y,
                (unsigned char)(std::min((int)(((unsigned short*)grid)[y * drawW + x] / darkenFactor), 255) * s[0] + c[0]),
                128 + c[1],
                128 + c[2]);
            }
          }
        }
      }
      SEND_DEBUG_IMAGE(shapes);
    }
  });
}

void ShapePerceptor::drawTheBalls(const unsigned char* circleHough,
                                  const unsigned int drawW,
                                  const unsigned int drawH,
                                  const unsigned int w,
                                  const unsigned int h,
                                  const bool isOptical) {

  DECLARE_DEBUG_DRAWING("module:ShapePerceptor:balls", "drawingOnImage"); // All ball candidates
  if (drawBalls && isOptical) {
    COMPLEX_DRAWING("module:ShapePerceptor:balls", {
      for (unsigned char i = 0; i < ballList.size(); ++i) {
        const int x = ballList[i] % w;
        const int y = ballList[i] / w;
        int r;
        ballRadiusInImage(Vector2<int>(x, y), downFactor, r);
        CIRCLE("module:ShapePerceptor:balls",
               x * drawW / w,
               y * drawH / h,
               r * drawW / w,
               1 * drawW / w,
               Drawings::ps_solid,
               ColorRGBA(0, 255, 0, circleHough[ballList[i]] + 32),
               Drawings::bs_null,
               ColorRGBA());
        DRAWTEXT("module:ShapePerceptor:balls",
                 x * downFactor * drawW / w,
                 y * downFactor * drawH / h,
                 10,
                 ColorRGBA(0, 255, 0),
                 circleHough[ballList[i]]);
      }
    });
  }
}

void ShapePerceptor::drawTheLines(const std::vector<ShapePercept::LineSegment>& segments,
                                  const unsigned int drawW,
                                  const unsigned int drawH,
                                  const unsigned int w,
                                  const unsigned int h,
                                  const bool isOptical) {

  DECLARE_DEBUG_DRAWING("module:ShapePerceptor:lines", "drawingOnImage"); // All field lines
  if (drawLines && isOptical) {
    COMPLEX_DRAWING("module:ShapePerceptor:lines", {
      for (unsigned int i = 0; i < segments.size(); ++i) {
        const Vector2<int>& p1 = segments[i].p1;
        const Vector2<int>& p2 = segments[i].p2;
        LINE("module:ShapePerceptor:lines",
             p1.x * drawW / w,
             p1.y * drawW / w,
             p2.x * drawW / w,
             p2.y * drawW / w,
             1 * drawW / w,
             Drawings::ps_solid,
             ColorRGBA(0, 127, 0));
        CROSS("module:ShapePerceptor:lines",
              (p1.x + p2.x) / 2 * drawW / w,
              (p1.y + p2.y) / 2 * drawH / h,
              1 * drawW / w,
              1,
              Drawings::ps_solid,
              ColorRGBA(0, 255, 0));
        if (labels == 1) {
          DRAWTEXT("module:ShapePerceptor:lines",
                   (p1.x + p2.x) / 2 * drawW / w,
                   (p1.y + p2.y) / 2 * drawH / h,
                   8,
                   ColorClasses::yellow,
                   segments[i].votes);
        } else if (labels == 2) {
          DRAWTEXT("module:ShapePerceptor:lines",
                   (p1.x + p2.x) / 2 * drawW / w,
                   (p1.y + p2.y) / 2 * drawH / h,
                   8,
                   ColorClasses::yellow,
                   segments[i].hits);
        }
      }
    });
  }
}

static unsigned char getBinaryAngle(const unsigned char* LUT, const signed short gx, const signed short gy) {
  if (gx > 0) {       // (192, 64)
    if (gy > 0) {     // (0, 64)
      if (gy <= gx) { // (0, 32]
        return LUT[gy * 255 / gx];
      } else { // (32, 64)
        return 64 - LUT[gx * 255 / gy];
      }
    } else if (gy < 0) { // (192, 256)
      if (-gy <= gx) {   // [224, 256)
        return 255 - LUT[-gy * 255 / gx] + 1;
      } else { // (192, 224)
        return LUT[-gx * 255 / gy] + 192;
      }
    } else { // 0
      return 0;
    }
  } else if (gx < 0) {
    if (gy > 0) {      // (64, 128)
      if (gy <= -gx) { // (96, 128)
        return 128 - LUT[-gy * 255 / gx];
      } else { // (64, 96)
        return LUT[-gx * 255 / gy] + 64;
      }
    } else if (gy < 0) { // (128, 192)
      if (-gy <= -gx) {  // (128, 160)
        return LUT[gy * 255 / gx] + 128;
      } else { // (160, 192)
        return 192 - LUT[gx * 255 / gy];
      }
    } else { // 128
      return 128;
    }
  } else if (gy > 0) { // 64, 192
    return 64;
  } else if (gy < 0) {
    return 192;
  }
  return 0; // gx = 0, gy = 0; invalid
}

// Angles are 8-bit (0 to 255)
static inline unsigned char angleDifference(const unsigned char a1, const unsigned char a2) {
  const unsigned char da = (a1 >= a2) ? (a1 - a2) : (a2 - a1);
  return std::min<unsigned char>(255 - da + 1, da);
}

// Returns a valid ball radius in px given downsized-image-y of center of ball, otherwise 0
inline bool ShapePerceptor::ballRadiusInImage(const int y, const int downFactor, int& radius) const {
  return ballRadiusInImage(Vector2<int>(theCameraInfo.width / 2 / downFactor, y), downFactor, radius);
}

// Returns a valid ball radius in px given downsized-image coordinates of center of ball, otherwise 0
bool ShapePerceptor::ballRadiusInImage(const Vector2<int>& centerInImageInt, const int downFactor, int& radius) const {
  const float worldRadius = theFieldDimensions.ballRadius;

  Vector2<> centerInImage(centerInImageInt.x * downFactor, centerInImageInt.y * downFactor);
  Vector3<> centerInSpace;
  if (!Geometry::calculatePointOnField(centerInImage, worldRadius, theCameraMatrix, theCameraInfo, centerInSpace)) {
    return false;
  }

  // HACK MAGIC Max dist on field
  if (centerInSpace.squareAbs() > 10400.f * 10400.f + 7400.f * 7400.f) {
    return false;
  }

  const Vector2<> centerOnField = Vector2<>(centerInSpace.x, centerInSpace.y);

  // toLeft points from ball center to "left side" of ball, as seen from robot
  Vector2<> toLeft = centerOnField;
  toLeft = toLeft.rotateLeft().normalize() * worldRadius;

  Vector3<> leftInSpace = centerInSpace + Vector3<>(toLeft.x, toLeft.y, 0.f);
  Vector3<> rightInSpace = centerInSpace + Vector3<>(-toLeft.x, -toLeft.y, 0.f);
  Vector2<> leftInImage;
  Vector2<> rightInImage;
  if (!Geometry::calculatePointInImage(leftInSpace, theCameraMatrix, theCameraInfo, leftInImage)) {
    return false;
  }
  if (!Geometry::calculatePointInImage(rightInSpace, theCameraMatrix, theCameraInfo, rightInImage)) {
    return false;
  }

  radius = (int)(0.5f * (leftInImage - rightInImage).abs()) / downFactor;
  return true;
}

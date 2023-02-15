/**
 * @file LineAnalyzer.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Perception/LineAnalysis.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/ShapePercept.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Configuration/FieldDimensions.h"

MODULE(LineAnalyzer)
REQUIRES(CameraMatrix)
REQUIRES(CameraInfo)
REQUIRES(LinePercept)
REQUIRES(ImageCoordinateSystem)
REQUIRES(FieldDimensions)
REQUIRES(FrameInfo)
PROVIDES_WITH_MODIFY_AND_OUTPUT(LineAnalysis)
END_MODULE

/**
 * @class LineAnalyzer
 * This class creates lines and intersection from the lineSpots. It takes
 * the nonLineSpots to form "BanSectors" to filter out lines seen in robots.
 */
class LineAnalyzer : public LineAnalyzerBase {
  /**
   * @class Parameters
   * The parameters for the LineAnalyzer.
   */
  STREAMABLE(
    Parameters,
    {
      ,
      (float)minWidthRatio, /**< The minimum height/width ratio a spot needs to be taken into account */
      (float)maxAlphaDiff,  /**< The maximum difference in direction (Hess norm form) for spots belonging to the same line */
      (float)maxDDiff,      /**< The maximum difference in distance (Hess norm form) for spots belonging to the same line */
      (int)minLineSingleRegionLength, /**< The minimum size of a region to form a line wihtout other the support of other
                                         spots */
      (int)minLineStartLength,        /**< The minimum size of a region to be able to look for supporters to forma line */
      (int)maxLineUniteDist,          /**< The maximum distance (Hess norm form) for two lines to be merge together */
      (float)maxLineUniteAlphaDiff,   /**< The maximum difference in direction (Hess norm form) for two lines to be merged
                                         together */
      (int)maxLineSingleSegDist,  /**< The maximum distance of the start and end point of a single segment to be added to a
                                     line */
      (int)maxLineSingleSegDist2, /**< The maximum disatnce of the start / end point to the start / end point of a line to be
                                     added to it */
      (int)maxLineDistance, /**< The maximum distance a line segment can be away from the camera to be taken into account */
      (float)minHardcover,  /**< The minimum hardcover ratio (length/covered by segments) of a line */
      (float)maxOverlapLength, /**< The maximum length two segments of a line may overlap */
      (int)minSupporters,      /**< The minimum of supporters a segments needs to become a line */
      (int)
        minTToEnd, /**< The minimum distance a intersection needs to the start / end of a line to be a T or X intersection */
      (int)
        maxTFromEnd, /**< The maximum distance a intersection can have to the start / end of a line to be a T and not a X */
      (float)minIntersectionAlphaDiff, /**< The minimum difference in direction two lines need to intersect */
      (int)minIntersectionLength,      /**< The minimum length of a line to be able to intersect another one */
      (int)maxMidLineToCircleDist,     /**< The maximum distance a line may have to the center of the center circle to be
                                          detected as the middle line */
      (int)minMidLineLength,           /**< The minimum length of a line to be accepted a middle line */
      (int)maxLineCircleDist,          /**< If a line is closer to the center circle than this, it will be deleted */
      (bool)rightAnglesOnly,           /**< Require lines to be at right angles to each other */
      (float)rightAngleTol,            /**< Keep lines within +/- this value (degrees) of the correct angle */
    });

  /**
   * @class NonLineParameters
   * Parameters to filter out lineSpots which are in BanSectors.
   */
  STREAMABLE(NonLineParameters,
             {
               ,
               (int)minLineLength,   /**< The minimum length of a spot to be filtered out */
               (float)maxAlphaDiff,  /**< The maximum direction offset of a spot from upright to be filtered out */
               (float)minWidthRatio, /**< The minimum height/width ratio of a spot to be filtered out */
             });

  /**
   * @class CircleParameters
   * Parameters for the center circle detection.
   */
  STREAMABLE(
    CircleParameters,
    {
      ,
      (int)maxNgbhDist,         /**< The maximum distance of two linesegments to create a circleSpot */
      (int)maxRadiusError,      /**< The maximum error in radius a intersection from two linesegments may have to create a
                                   circleSpot */
      (int)minSegmentLength,    /**< The minimum length of a lineSegment to be taken into account for the center circle */
      (int)minSegmentImgLength, /**< The minimum length in image coordinates of a linesegment to be taken into acount for the
                                   center circle */
      (int)minSupporters,       /**< The minimum number of supporters for the center circle */
      (int)maxSupporterDist,    /**< The maximum distance of two circleSpots to support each other */
      (int)maxSupporterDist2,   /**< The maximum distance of two circleSpots to support each other in the second round */
    });

  /**
   * @class BanSectorParameters
   * Parameters for the creation of the BanSectors.
   */
  STREAMABLE(
    BanSectorParameters,
    {
      ,
      (float)angleStepSize,    /**< A angle added to the left and right of a sector (so a sector is bigger than the detected
                                  nonLineSpots) */
      (int)minSectorCounter,   /**< The minimum number of spots in a sector to be a BanSector */
      (float)maxLineAngleDiff, /**< The maximum distance (angle) a line may have from the sector to be filtered out */
    });

  /**
   * @class ParameterWrapper
   * A class which wrapps the different parameter classes for the usage of a config map
   */
  STREAMABLE(ParameterWrapper,
             {
               ,
               (Parameters)(parameters)parameters,
               (CircleParameters)(circleParams)circleParams,
               (NonLineParameters)(nonLineParams)nonLineParams,
               (BanSectorParameters)(banSectorParams)banSectorParams,
             });

  /**
   * @class BanSector
   * A class to hold a BanSector.
   */
  class BanSector {
  public:
    int start;        /**< Distance the Sector starts */
    int end;          /**<Distance the sector ends */
    float alphaLeft;  /**< left angle of the sector */
    float alphaRight; /**< right angle of the sector */
    int counter;      /**< Number of nonLineSpots in this sector */
  };

  Parameters parameters;               /**< Parameters for this module */
  CircleParameters circleParams;       /**< Parameters for center circle detection */
  NonLineParameters nonLineParams;     /**< Parameters for filtering out lines near robots */
  BanSectorParameters banSectorParams; /**< Parameters for the creation of ban sectors */

  std::list<LineAnalysis::LineSegment> lineSegs; /**< All the lineSegments */
  std::list<LineAnalysis::LineSegment> singleSegs;
  std::list<LineAnalysis::ObservedFieldLine> lines;
  std::list<BanSector> banSectors; /**< The ban sectors, where no vertical, long spots are accepted */

  /** update the LineAnalysis */
  void update(LineAnalysis& lineAnalysis);

  /** create BanSectors */
  // void createBanSectors();

  /**
   * creates the linesegments from the lineSpots, it creates the BanSector and filters out lines which meet
   * the banSector filter criterions
   * @param singleSegs a reference to the singleSegs list in the LineAnalysis
   * */
  // void createLineSegments(std::list<LineAnalysis::LineSegment>& singleSegs);

  /**
   * takes in linesegments provided in LinePercept; does not create BanSectors
   * @param input a reference to the singleSegs vector in the LinePercept
   * @param output a reference to the singleSegs list in the LineAnalysis
   */
  void importLineSegments(const std::vector<ShapePercept::LineSegment>& inputs,
                          std::list<LineAnalysis::LineSegment>& outputs);

  /**
   * creates the lines from the singleSegments
   * @param lines a reference to the lines list in the LineAnalysis
   * @param singleSegs a reference to the singleSegs list in the LineAnalysis
   * */
  void createLines(std::list<LineAnalysis::ObservedFieldLine>& lines, std::list<LineAnalysis::LineSegment>& singleSegs);

  /**
   * keep only lines at approximately right angles to each other inside lines
   * and put the outliers back into singleSegs
   * @param lines a reference to the lines list in the LineAnalysis
   * @param singleSegs a reference to the singleSegs list in the LineAnalysis
   */
  void keepRightAngledLines(std::list<LineAnalysis::ObservedFieldLine>& lines,
                            std::list<LineAnalysis::LineSegment>& singleSegs);

  /**
   * analyzes the lines, merges and deletes some if neccessary and creates intersections
   * @param lines a reference to the lines list in the LineAnalysis
   * @param intersections a reference to the intersections vector in the LineAnalysis
   * @param circle a reference to the circle in the LineAnalysis
   * @param singleSegs a reference to the singleSegs list in the LineAnalysis
   * */
  void analyzeLines(std::list<LineAnalysis::ObservedFieldLine>& lines,
                    std::vector<LineAnalysis::Intersection>& intersections,
                    LineAnalysis::CircleSpot& circle,
                    std::list<LineAnalysis::LineSegment>& singleSegs);

  /**
   * analyze the singleSegments and try to find the center circle
   * @param singleSegs a reference to the singleSegs list in the LineAnalysis
   * @param circle a reference to the circle in the LineAnalysis
   * @param lines a reference to the lines list in the LineAnalysis
   * */
  void analyzeSingleSegments(std::list<LineAnalysis::LineSegment>& singleSegs,
                             LineAnalysis::CircleSpot& circle,
                             std::list<LineAnalysis::ObservedFieldLine>& lines);

  /**
   * Determines the start and end point of a line. If updateLine == true the d and alpha values of the line
   * (Hess norm form) are recalculated.
   * @param line the line to determine the start and end point of
   * @param first returns the start/end point of the line
   * @param last returns the start/end point of the line
   * @param updateLine recalculate d and alpha of line (Hess norm form)?
   * */
  void
  getFirstAndLastOfLine(LineAnalysis::ObservedFieldLine& line, Vector2<>& first, Vector2<>& last, bool updateLine = true);

public:
  /*
   * Default constructor
   */
  LineAnalyzer();
};

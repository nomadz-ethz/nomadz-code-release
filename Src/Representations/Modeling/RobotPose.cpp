/**
 * @file RobotPose.cpp
 *
 * The file contains the implementation of the streaming operators
 * for the class RobotPose
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#include "RobotPose.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Debugging/DebugDrawings3D.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"

void RobotPose::draw(bool teamRed) {
  DECLARE_DEBUG_DRAWING("representation:RobotPose", "drawingOnField");
  Vector2<> bodyPoints[4] = {Vector2<>(55, 90), Vector2<>(-55, 90), Vector2<>(-55, -90), Vector2<>(55, -90)};
  for (int i = 0; i < 4; i++) {
    bodyPoints[i] = *this * bodyPoints[i];
  }
  Vector2<> dirVec(200, 0);
  dirVec = *this * dirVec;
  const ColorRGBA ownTeamColorForDrawing = teamRed ? ColorRGBA(255, 0, 0) : ColorRGBA(0, 0, 255);
  LINE("representation:RobotPose",
       translation.x,
       translation.y,
       dirVec.x,
       dirVec.y,
       20,
       Drawings::ps_solid,
       ColorClasses::white);
  POLYGON("representation:RobotPose",
          4,
          bodyPoints,
          20,
          Drawings::ps_solid,
          ownTeamColorForDrawing,
          Drawings::bs_solid,
          ColorClasses::white);
  CIRCLE("representation:RobotPose",
         translation.x,
         translation.y,
         42,
         0,
         Drawings::ps_solid,
         ownTeamColorForDrawing,
         Drawings::bs_solid,
         ownTeamColorForDrawing);

  DECLARE_DEBUG_DRAWING("representation:RobotPose:deviation", "drawingOnField");
  if (deviation < 100000.f) {
    DRAWTEXT(
      "representation:RobotPose:deviation", -3000, -2300, 12, ColorRGBA(0xff, 0xff, 0xff), "pose deviation: " << deviation);
  } else {
    DRAWTEXT("representation:RobotPose:deviation", -3000, -2300, 12, ColorRGBA(0xff, 0xff, 0xff), "pose deviation: unknown");
  }

  DECLARE_DEBUG_DRAWING3D("representation:RobotPose", "field", {
    LINE3D("representation:RobotPose", translation.x, translation.y, 10, dirVec.x, dirVec.y, 10, 2, ownTeamColorForDrawing);
    DRAWTEXT("representation:RobotPose",
             translation.x + 150,
             translation.y,
             9,
             ColorRGBA(0x00, 0x00, 0x00),
             "rot: " + std::to_string(rotation).substr(0, 5));
    DRAWTEXT("representation:RobotPose",
             translation.x + 150,
             translation.y + 150,
             9,
             ColorRGBA(0x00, 0x00, 0x00),
             "y: " + std::to_string(translation.y).substr(0, 5));
    DRAWTEXT("representation:RobotPose",
             translation.x + 150,
             translation.y + 300,
             9,
             ColorRGBA(0x00, 0x00, 0x00),
             "x: " + std::to_string(translation.x).substr(0, 5));

    for (int i = 0; i < 4; ++i) {
      const Vector2<> p1 = bodyPoints[i];
      const Vector2<> p2 = bodyPoints[(i + 1) & 3];
      LINE3D("representation:RobotPose", p1.x, p1.y, 10, p2.x, p2.y, 10, 2, ownTeamColorForDrawing);
    }
  });

  DECLARE_DEBUG_DRAWING("origin:RobotPose", "drawingOnField"); // Set the origin to the robot's current position
  DECLARE_DEBUG_DRAWING("origin:RobotPoseWithoutRotation", "drawingOnField");
  ORIGIN("origin:RobotPose", translation.x, translation.y, rotation);
  ORIGIN("origin:RobotPoseWithoutRotation", translation.x, translation.y, 0);
}

void RobotPose::drawOnImage(const CameraMatrix& theCameraMatrix,
                            const CameraInfo& theCameraInfo,
                            const FieldDimensions& theFieldDimensions,
                            const ColorRGBA color) {
  DECLARE_DEBUG_DRAWING("representation:RobotPose:Image", "drawingOnImage");

  COMPLEX_DRAWING("representation:RobotPose:Image", {
    const Pose2D poseInv = this->invert();
    const float& f = theCameraInfo.focalLength;
    const Pose3D theCameraMatrixInv = theCameraMatrix.invert();

    // Draw field lines
    for (auto spec : theFieldDimensions.getAllFieldLines()) {

      spec.corner(poseInv + spec.corner());
      auto lineOnField = Geometry::Line::fromPoints(spec.from, spec.to);

      Vector2<> p1 = lineOnField.base;
      Vector2<> p2 = p1 + lineOnField.direction;
      Vector3<> p1Camera(p1.x, p1.y, 0);
      Vector3<> p2Camera(p2.x, p2.y, 0);

      // points are transformed into camera coordinates
      p1Camera = theCameraMatrixInv * p1Camera;
      p2Camera = theCameraMatrixInv * p2Camera;

      // handle the case that points can lie behind the camera plane
      bool p1Behind = p1Camera.x < f;
      bool p2Behind = p2Camera.x < f;
      if (p1Behind && p2Behind) {
        continue;
      } else if (!p1Behind && !p2Behind) {
        // both rays can be simply intersected with the image plane
        p1Camera /= (p1Camera.x / f);
        p2Camera /= (p2Camera.x / f);
      } else {
        // if one point lies behind the camera and the other in front, there must be an intersection of the connective line
        // with the image plane
        const Vector3<> direction = p1Camera - p2Camera;
        const float scale = (f - p1Camera.x) / direction.x;
        const Vector3<> intersection = p1Camera + direction * scale;
        if (p1Behind) {
          p1Camera = intersection;
          p2Camera /= (p2Camera.x / f);
        } else {
          p2Camera = intersection;
          p1Camera /= (p1Camera.x / f);
        }
      }
      Vector2<> p1Result(theCameraInfo.opticalCenter.x - p1Camera.y, theCameraInfo.opticalCenter.y - p1Camera.z);
      Vector2<> p2Result(theCameraInfo.opticalCenter.x - p2Camera.y, theCameraInfo.opticalCenter.y - p2Camera.z);

      Geometry::Line lineInImage;
      lineInImage.base = p1Result;
      lineInImage.direction = p2Result - p1Result;

      Vector2<> startPoint = lineInImage.base;
      Vector2<> endPoint = startPoint + lineInImage.direction;
      LINE(
        "representation:RobotPose:Image", startPoint.x, startPoint.y, endPoint.x, endPoint.y, 3, Drawings::ps_solid, color);
    }
  });
}

void GroundTruthRobotPose::draw() const {
  DECLARE_DEBUG_DRAWING("representation:GroundTruthRobotPose", "drawingOnField");
  ColorRGBA transparentWhite(ColorClasses::white);
  transparentWhite.a = 128;
  Vector2<> bodyPoints[4] = {Vector2<>(55, 90), Vector2<>(-55, 90), Vector2<>(-55, -90), Vector2<>(55, -90)};
  for (int i = 0; i < 4; i++) {
    bodyPoints[i] = *this * bodyPoints[i];
  }
  Vector2<> dirVec(200, 0);
  dirVec = *this * dirVec;
  const ColorRGBA ownTeamColorForDrawing(0, 0, 0, 128);
  LINE("representation:GroundTruthRobotPose",
       translation.x,
       translation.y,
       dirVec.x,
       dirVec.y,
       20,
       Drawings::ps_solid,
       transparentWhite);
  POLYGON("representation:GroundTruthRobotPose",
          4,
          bodyPoints,
          20,
          Drawings::ps_solid,
          ownTeamColorForDrawing,
          Drawings::bs_solid,
          transparentWhite);
  CIRCLE("representation:GroundTruthRobotPose",
         translation.x,
         translation.y,
         42,
         0,
         Drawings::ps_solid,
         ownTeamColorForDrawing,
         Drawings::bs_solid,
         ownTeamColorForDrawing);

  DECLARE_DEBUG_DRAWING("origin:GroundTruthRobotPose",
                        "drawingOnField"); // Set the origin to the robot's ground truth position
  DECLARE_DEBUG_DRAWING("origin:GroundTruthRobotPoseWithoutRotation", "drawingOnField");
  ORIGIN("origin:GroundTruthRobotPose", translation.x, translation.y, rotation);
  ORIGIN("origin:GroundTruthRobotPoseWithoutRotation", translation.x, translation.y, 0);
}

RobotPoseSamples::RobotPoseSample::RobotPoseSample(float x, float y, float angle, float validity, float deviation)
    : RobotPoseSample() {
  this->x = x;
  this->y = y;
  this->angle = angle;
  this->validity = validity;
  this->deviation = deviation;
}

void RobotPoseSamples::draw() const {
  for (const auto& sample : samples) {
    const Pose2D pose(sample.angle, sample.x, sample.y);
    DECLARE_DEBUG_DRAWING("representation:RobotPoseSamples", "drawingOnField");
    Vector2<> bodyPoints[4] = {{55.f, 90.f}, {-55.f, 90.f}, {-55.f, -90.f}, {55.f, -90.f}};

    // Transform bodyPoints to field frame
    for (int i = 0; i < 4; i++) {
      bodyPoints[i] = pose * bodyPoints[i];
    }

    Vector2<> dirVec(200, 0);
    dirVec = pose * dirVec;

    const unsigned char gray = (unsigned char)(255.f * (1.f - sample.validity));
    const ColorRGBA color = ColorRGBA(gray, gray, gray);

    LINE("representation:RobotPoseSamples",
         pose.translation.x,
         pose.translation.y,
         dirVec.x,
         dirVec.y,
         20,
         Drawings::ps_solid,
         ColorClasses::white);
    POLYGON("representation:RobotPoseSamples",
            4,
            bodyPoints,
            20,
            Drawings::ps_solid,
            color,
            Drawings::bs_solid,
            ColorClasses::white);
    CIRCLE("representation:RobotPoseSamples",
           pose.translation.x,
           pose.translation.y,
           42,
           0,
           Drawings::ps_solid,
           color,
           Drawings::bs_solid,
           color);
    DRAWTEXT("representation:RobotPoseSamples",
             pose.translation.x,
             pose.translation.y,
             9,
             ColorClasses::yellow,
             (int)std::round(sample.validity * 100.f));
  }
}

RobotPoseCompressed::RobotPoseCompressed(const RobotPose& robotPose)
    : translation(robotPose.translation), deviation(robotPose.deviation) {
  float normalizedAngle = normalize(robotPose.rotation);
  int discretizedAngle = (int)(normalizedAngle * 128.0f / pi);
  if (discretizedAngle > 127) {
    discretizedAngle = -128;
  }
  rotation = (char)discretizedAngle;
  validity = (unsigned char)(robotPose.validity * 255.f);
}

RobotPoseCompressed::operator RobotPose() const {
  RobotPose robotPose;
  robotPose.translation = Vector2<>(translation);
  robotPose.rotation = (float)rotation * pi / 128.f;
  robotPose.validity = (float)validity / 255.f;
  robotPose.deviation = deviation;
  return robotPose;
}

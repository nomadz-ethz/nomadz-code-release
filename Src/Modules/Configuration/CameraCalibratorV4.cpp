/**
 * @file CameraCalibratorV4.cpp
 *
 * This file implements a module that can provide a semiautomatic camera calibration.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Alexander Härtl
 */

#include "CameraCalibratorV4.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"
#include "Core/Streams/InStreams.h"
#include "Core/Settings.h"
#include <limits>

using namespace std;

const float invalidNumber = -10000.f;
const Vector2<float> invalidPoint(invalidNumber, invalidNumber); // also works for Vector2<int> (implicitly cast)

CameraCalibratorV4::CameraCalibratorV4()
    : calibrationState(Idle), lastFetchedPoint(-1, -1), drawMode(DrawMode::line), startedDrawingLine(false), optimizer(0),
      currentCamera(CameraInfo::lower) {}

CameraCalibratorV4::~CameraCalibratorV4() {
  if (optimizer) {
    delete optimizer;
  }
}

void CameraCalibratorV4::update(RobotPose& robotPose) {
  Pose2D inputRobotPose;
  MODIFY("module:CameraCalibratorV4:robotPose", inputRobotPose);
  if (inputRobotPose != lastFetchedRobotPose) {
    this->robotPose = lastFetchedRobotPose = inputRobotPose;
  }
  robotPose = this->robotPose;
  robotPose.validity = 1;
  robotPose.deviation = RobotPose::unknownDeviation;
}

void CameraCalibratorV4::update(CameraCalibration& cameraCalibration) {
  RobotCameraMatrix robotCameraMatrix(theRobotDimensions,
                                      theFilteredJointData.angles[JointData::HeadYaw],
                                      theFilteredJointData.angles[JointData::HeadPitch],
                                      cameraCalibration,
                                      theCameraInfo.camera == CameraInfo::upper);
  cameraMatrix.computeCameraMatrix(theTorsoMatrix, robotCameraMatrix, cameraCalibration);

  if (theCameraInfo.camera == CameraInfo::upper) {
    upperCameraInfo = theCameraInfo;
  } else if (theCameraInfo.camera == CameraInfo::lower) {
    lowerCameraInfo = theCameraInfo;
  }

  this->cameraCalibration = &cameraCalibration;

  // Select camera for current point selection
  MODIFY("module:CameraCalibratorV4:currentCamera", currentCamera);

  // What to draw on click
  DEBUG_RESPONSE_ONCE("module:CameraCalibratorV4:lineMode", {
    startedDrawingLine = false;
    drawMode = line;
  });
  DEBUG_RESPONSE_ONCE("module:CameraCalibratorV4:pointMode", { drawMode = point; });

  // this is the interface to the ImageView of the simulator
  Vector2<int> point(-1, -1);
  MODIFY("module:CameraCalibrator:point", point);

  // "declare" debug responses. This is necessary to be able to send out
  // the debug requests without the necessity to poll after every state change
  if (calibrationState != Idle) {
    DEBUG_RESPONSE_ONCE("module:CameraCalibratorV4:collectPoints", );
  }
  if (calibrationState != Accumulate) {
    DEBUG_RESPONSE_ONCE("module:CameraCalibratorV4:optimize", );
    DEBUG_RESPONSE_ONCE("module:CameraCalibratorV4:undo", );
    DEBUG_RESPONSE_ONCE("module:CameraCalibratorV4:clear", );
  }
  if (calibrationState != Optimize) {
    DEBUG_RESPONSE_ONCE("module:CameraCalibratorV4:stop", );
  }

  // the optimizer is only needed during the state Optimize
  if (calibrationState != Optimize && optimizer) {
    delete optimizer;
    optimizer = 0;
  }

  switch (calibrationState) {
  case (Idle):
    DEBUG_RESPONSE_ONCE("module:CameraCalibratorV4:collectPoints", {
      calibrationState = Accumulate;
      MODIFY("module:CameraCalibrator:point", lastFetchedPoint);
    });
    break;
  case (Accumulate): // collect points used for the optimization
    fetchPoint();
    DEBUG_RESPONSE_ONCE("module:CameraCalibratorV4:undo", { samples.pop_back(); });
    DEBUG_RESPONSE_ONCE("module:CameraCalibratorV4:clear", {
      MODIFY("module:CameraCalibrator:point", lastFetchedPoint);
      samples.clear();
    });
    DEBUG_RESPONSE_ONCE("module:CameraCalibratorV4:optimize", {
      if (!(samples.size() > numOfParameterTranslations))
        OUTPUT(
          idText, text, "CameraCalibratorV4: Error! too few samples! need " << numOfParameterTranslations << " or more");
      else
        calibrationState = Optimize;
    });
    break;
  case (Optimize): // optimize the parameters of the cameraCalibration
    if (!optimizer) {
      vector<float> initialParameters;
      initialParameters.resize(calibrateBothCameras ? numOfParameterTranslations : numOfParametersLowerCamera);
      // since the parameters for the robot pose are correction parameters, an empty RobotPose is used instead of
      // theRobotPose
      translateParameters(cameraCalibration, RobotPose(), initialParameters);
      optimizer = new GaussNewtonOptimizer<Sample, CameraCalibratorV4>(
        initialParameters, samples, *this, &CameraCalibratorV4::computeErrorParameterVector);
      successfulConvergences = 0;
      framesToWait = 0;
    } else {
      DEBUG_RESPONSE_ONCE("module:CameraCalibratorV4:stop", calibrationState = Idle;);
      // only do an iteration after some frames have passed
      if (framesToWait <= 0) {
        framesToWait = numOfFramesToWait;
        const float delta = optimizer->iterate();
        OUTPUT(idText, text, "CameraCalibratorV4: delta = " << delta);

        // the camera calibration is refreshed from the current optimizer state
        RobotPose robotPose;
        const vector<float> currentParameters = optimizer->getParameters();
        translateParameters(currentParameters, cameraCalibration, robotPose);

        if (abs(delta) < terminationCriterion) {
          ++successfulConvergences;
        }
        if (successfulConvergences >= minSuccessfulConvergences) {
          calibrationState = Idle;
          OUTPUT(idText, text, "CameraCalibratorV4: converged!");
          OUTPUT(idText,
                 text,
                 "RobotPoseCorrection: x = " << currentParameters[robotPoseCorrectionX] * 1000.0f
                                             << ", y = " << currentParameters[robotPoseCorrectionY] * 1000.0f
                                             << ", rot = " << currentParameters[robotPoseCorrectionRot]);
          this->robotPose.translation.x += currentParameters[robotPoseCorrectionX] * 1000.0f;
          this->robotPose.translation.y += currentParameters[robotPoseCorrectionY] * 1000.0f;
          this->robotPose.rotation = normalize(this->robotPose.rotation + currentParameters[robotPoseCorrectionRot]);
        }
      }
      --framesToWait;
    }
    break;
  default:
    ASSERT(false);
  }

  DECLARE_DEBUG_DRAWING("module:CameraCalibratorV4:drawFieldLines", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CameraCalibratorV4:drawSamples", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CameraCalibratorV4:points", "drawingOnImage", {
    std::string statusText;
    switch (calibrationState) {
    case Idle:
      statusText = "(idle)";
      break;
    case Accumulate:
      switch (drawMode) {
      case DrawMode::point:
        statusText = "(click to draw point)";
        break;
      case DrawMode::line:
        if (!startedDrawingLine) {
          statusText = "(click to start line)";
        } else {
          statusText = "(click to end line)";
        }
        break;
      }
      break;
    case Optimize:
      statusText = "(optimizing)";
      break;
    }
    ColorClasses::Color color = !(samples.size() > numOfParameterTranslations) ? ColorClasses::red : ColorClasses::green;
    DRAWTEXT("module:CameraCalibratorV4:points",
             5,
             15,
             20,
             color,
             "Points collected: " << (unsigned)samples.size() << " " << statusText);
    if (calibrationState == Accumulate && theCameraInfo.camera != currentCamera) {
      DRAWTEXT("module:CameraCalibratorV4:points",
               5,
               25,
               11,
               color,
               "Please switch: collecting on " << ((currentCamera == CameraInfo::upper) ? "upper" : "lower") << " cam");
    }
  });

  COMPLEX_DRAWING("module:CameraCalibratorV4:drawFieldLines", drawFieldLines(););
  if (theCameraInfo.camera == currentCamera) {
    COMPLEX_DRAWING("module:CameraCalibratorV4:drawSamples", drawSamples(););
  }
}

float CameraCalibratorV4::computeError(const Sample& sample,
                                       const CameraCalibration& cameraCalibration,
                                       const RobotPose& robotPose,
                                       bool inImage) const {
  // a sample of the upper camera is ignored if only the lower camera is calibrated
  if (!calibrateBothCameras && sample.isUpperCamera) {
    return 0.0f;
  }

  // build camera matrix from sample and camera calibration
  const RobotCameraMatrix robotCameraMatrix(
    theRobotDimensions, sample.headYaw, sample.headPitch, cameraCalibration, sample.isUpperCamera);
  const CameraMatrix cameraMatrix(sample.torsoMatrix, robotCameraMatrix, cameraCalibration);
  const CameraInfo& cameraInfo = sample.isUpperCamera ? upperCameraInfo : lowerCameraInfo;

  if (inImage) {
    const Pose2D robotPoseInv = robotPose.invert();
    float minimum = numeric_limits<float>::max();
    for (auto lineOnField : theFieldDimensions.getAllFieldLines()) {
      // transform the line in robot relative coordinates
      lineOnField.corner(robotPoseInv + lineOnField.corner());
      Geometry::Line lineInImage;
      float distance;
      if (!projectLineOnFieldIntoImage(
            Geometry::Line::fromPoints(lineOnField.from, lineOnField.to), cameraMatrix, cameraInfo, lineInImage)) {
        distance = aboveHorizonError;
      } else {
        distance = Geometry::getDistanceToEdge(lineInImage, Vector2<>(sample.pointInImage));
      }
      if (distance < minimum) {
        minimum = distance;
      }
    }
    return minimum;
  } else // on ground
  {
    // project point in image onto ground
    Vector3<> cameraRay(cameraInfo.focalLength,
                        cameraInfo.opticalCenter.x - sample.pointInImage.x,
                        cameraInfo.opticalCenter.y - sample.pointInImage.y);
    cameraRay = cameraMatrix * cameraRay;
    if (cameraRay.z >= 0) // above horizon
    {
      return aboveHorizonError;
    }
    const float scale = cameraMatrix.translation.z / -cameraRay.z;
    cameraRay *= scale;
    Vector2<> pointOnGround(cameraRay.x, cameraRay.y); // point on ground relative to the robot
    pointOnGround = robotPose * pointOnGround;         // point on ground in absolute coordinates

    float minimum = numeric_limits<float>::max();
    for (const auto& fieldLine : theFieldDimensions.getAllFieldLines()) {
      auto line = Geometry::Line::fromPoints(fieldLine.from, fieldLine.to);
      const float distance = Geometry::getDistanceToEdge(line, pointOnGround);
      if (distance < minimum) {
        minimum = distance;
      }
    }
    return minimum;
  }
}

float CameraCalibratorV4::computeErrorParameterVector(const Sample& sample, const vector<float>& parameters) const {
  CameraCalibration cameraCalibration = *this->cameraCalibration;
  RobotPose robotPose;
  translateParameters(parameters, cameraCalibration, robotPose);

  // the correction parameters for the robot pose are added to theRobotPose
  // in the parameter space the robot pose translation unit is m to keep the order of magnitude similar to the other
  // parameters
  robotPose.translation *= 1000.0f;
  robotPose.translation += this->robotPose.translation;
  robotPose.rotation += this->robotPose.rotation;

  return computeError(sample, cameraCalibration, robotPose);
}

void CameraCalibratorV4::translateParameters(const vector<float>& parameters,
                                             CameraCalibration& cameraCalibration,
                                             RobotPose& robotPose) const {
  ASSERT(parameters.size() == numOfParameterTranslations || parameters.size() == numOfParametersLowerCamera);

  cameraCalibration.cameraTiltCorrection = parameters[cameraTiltCorrection];
  cameraCalibration.cameraRollCorrection = parameters[cameraRollCorrection];
  cameraCalibration.bodyTiltCorrection = parameters[bodyTiltCorrection];
  cameraCalibration.bodyRollCorrection = parameters[bodyRollCorrection];

  robotPose.translation.x = parameters[robotPoseCorrectionX];
  robotPose.translation.y = parameters[robotPoseCorrectionY];
  robotPose.rotation = parameters[robotPoseCorrectionRot];

  if (parameters.size() == numOfParameterTranslations) {
    cameraCalibration.upperCameraRollCorrection = parameters[upperCameraX];
    cameraCalibration.upperCameraTiltCorrection = parameters[upperCameraY];
    cameraCalibration.upperCameraPanCorrection = parameters[upperCameraZ];
  }
}

void CameraCalibratorV4::translateParameters(const CameraCalibration& cameraCalibration,
                                             const RobotPose& robotPose,
                                             vector<float>& parameters) const {
  ASSERT(parameters.size() == numOfParameterTranslations || parameters.size() == numOfParametersLowerCamera);

  parameters[cameraTiltCorrection] = cameraCalibration.cameraTiltCorrection;
  parameters[cameraRollCorrection] = cameraCalibration.cameraRollCorrection;
  parameters[bodyTiltCorrection] = cameraCalibration.bodyTiltCorrection;
  parameters[bodyRollCorrection] = cameraCalibration.bodyRollCorrection;

  parameters[robotPoseCorrectionX] = robotPose.translation.x;
  parameters[robotPoseCorrectionY] = robotPose.translation.y;
  parameters[robotPoseCorrectionRot] = robotPose.rotation;

  if (parameters.size() == numOfParameterTranslations) {
    parameters[upperCameraX] = cameraCalibration.upperCameraRollCorrection;
    parameters[upperCameraY] = cameraCalibration.upperCameraTiltCorrection;
    parameters[upperCameraZ] = cameraCalibration.upperCameraPanCorrection;
  }
}

void CameraCalibratorV4::fetchPoint() {

  Vector2<int> point(invalidPoint);
  MODIFY("module:CameraCalibrator:point", point);

  if (theCameraInfo.camera != currentCamera) {
    return;
  }

  if (point == lastFetchedPoint) {
    return;
  }

  Vector2<> pointOnField;
  if (!Geometry::calculatePointOnField(point.x, point.y, cameraMatrix, theCameraInfo, pointOnField)) {
    OUTPUT(idText,
           text,
           "Point (" << point.x << ", " << point.y << ") not on field ("
                     << (theCameraInfo.camera == CameraInfo::upper ? "upper " : "lower ") << "cam)");
    lastFetchedPoint = point;
    return;
  }

  bool lastSampleExists = false;
  Sample lastSample;
  if (samples.size()) {
    lastSampleExists = true;
    lastSample = samples.back();
  }

  // Move last point
  const int moveThreshold = (theCameraInfo.camera == CameraInfo::upper) ? 10 : 5;
  if (lastSampleExists && (lastSample.pointInImage - point).abs() < moveThreshold) {
    samples.pop_back();
    startedDrawingLine = false; // We're not completing a line here: reset drawing status
  }

  if (drawMode == DrawMode::line) {

    // Sanity check
    const bool isUpperCamera = (theCameraInfo.camera == CameraInfo::upper);
    if (lastSampleExists && lastSample.isUpperCamera != isUpperCamera) {
      startedDrawingLine = false;
    }

    if (startedDrawingLine) {
      // Complete the line
      const int minSegmentLength =
        (theCameraInfo.camera == CameraInfo::upper) ? minSegmentLengthUpper : minSegmentLengthLower; // px
      const Sample lastSample = samples.back();

      // Previous point might have been taken from different camera angle
      Vector2<int> drawLineFrom;
      Geometry::calculatePointInImage(
        Vector3<>(lastSample.pointOnField.x, lastSample.pointOnField.y, 0.f), cameraMatrix, theCameraInfo, drawLineFrom);
      Vector2<int> drawLineTo = point;

      // There will be (numSegments) segments, or (numSegments + 1) points, incl. start & end points
      const int numSegments = (int)((drawLineTo - drawLineFrom).abs() / minSegmentLength);

      if (numSegments > 0) {
        for (int seg = 1; seg < numSegments;
             ++seg) // only middle points: start was added on prev click; end will be added near end of function
        {
          Vector2<int> newPoint;
          newPoint.x = drawLineFrom.x + (drawLineTo.x - drawLineFrom.x) * seg / numSegments;
          newPoint.y = drawLineFrom.y + (drawLineTo.y - drawLineFrom.y) * seg / numSegments;
          addPoint(newPoint);
        }
      }

      startedDrawingLine = false;
    } else {
      // Complete line on next new point
      startedDrawingLine = true;
    }
  }

  addPoint(point);
  lastFetchedPoint = point;
}

// Checks (& returns) if point is valid, and adds it to list of samples
bool CameraCalibratorV4::addPoint(const Vector2<int>& point) {
  // store all necessary information in the sample
  Vector2<> pointOnField(invalidPoint);
  if (!Geometry::calculatePointOnField(point.x, point.y, cameraMatrix, theCameraInfo, pointOnField)) {
    OUTPUT(idText,
           text,
           "Point (" << point.x << ", " << point.y << ") not on field ("
                     << (theCameraInfo.camera == CameraInfo::upper ? "upper " : "lower ") << "cam)");
    pointOnField = Vector2<>(invalidPoint);
    return false;
  }
  Sample sample;
  sample.pointInImage = point;
  sample.pointOnField = pointOnField; // for drawing
  sample.torsoMatrix = theTorsoMatrix;
  sample.headYaw = theFilteredJointData.angles[JointData::HeadYaw];
  sample.headPitch = theFilteredJointData.angles[JointData::HeadPitch];
  sample.isUpperCamera = theCameraInfo.camera == CameraInfo::upper;

  samples.push_back(sample);
  return true;
}

bool CameraCalibratorV4::projectLineOnFieldIntoImage(const Geometry::Line& lineOnField,
                                                     const CameraMatrix& cameraMatrix,
                                                     const CameraInfo& cameraInfo,
                                                     Geometry::Line& lineInImage) const {
  const float& f = cameraInfo.focalLength;
  const Pose3D cameraMatrixInv = cameraMatrix.invert();

  // TODO more elegant solution directly using the direction of the line?

  // start and end point of the line
  Vector2<> p1 = lineOnField.base;
  Vector2<> p2 = p1 + lineOnField.direction;
  Vector3<> p1Camera(p1.x, p1.y, 0);
  Vector3<> p2Camera(p2.x, p2.y, 0);

  // points are transformed into camera coordinates
  p1Camera = cameraMatrixInv * p1Camera;
  p2Camera = cameraMatrixInv * p2Camera;

  // handle the case that points can lie behind the camera plane
  const bool p1Behind = p1Camera.x < cameraInfo.focalLength;
  const bool p2Behind = p2Camera.x < cameraInfo.focalLength;
  if (p1Behind && p2Behind) {
    return false;
  } else if (!p1Behind && !p2Behind) {
    // both rays can be simply intersected with the image plane
    p1Camera /= (p1Camera.x / f);
    p2Camera /= (p2Camera.x / f);
  } else {
    // if one point lies behind the camera and the other in front, there must be an intersection of the connective line with
    // the image plane
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
  const Vector2<> p1Result(cameraInfo.opticalCenter.x - p1Camera.y, cameraInfo.opticalCenter.y - p1Camera.z);
  const Vector2<> p2Result(cameraInfo.opticalCenter.x - p2Camera.y, cameraInfo.opticalCenter.y - p2Camera.z);
  lineInImage.base = p1Result;
  lineInImage.direction = p2Result - p1Result;
  return true;
}

void CameraCalibratorV4::drawFieldLines() {
  const Pose2D robotPoseInv = robotPose.invert();
  for (auto lineOnField : theFieldDimensions.getAllFieldLines()) {
    lineOnField.corner(robotPoseInv + lineOnField.corner());
    Geometry::Line lineInImage;
    if (projectLineOnFieldIntoImage(
          Geometry::Line::fromPoints(lineOnField.from, lineOnField.to), cameraMatrix, theCameraInfo, lineInImage)) {
      LINE("module:CameraCalibratorV4:drawFieldLines",
           lineInImage.base.x,
           lineInImage.base.y,
           (lineInImage.base + lineInImage.direction).x,
           (lineInImage.base + lineInImage.direction).y,
           1,
           Drawings::ps_solid,
           ColorClasses::black);
    }
  }
}

void CameraCalibratorV4::drawSamples() {
  for (vector<Sample>::const_iterator s = samples.begin(); s != samples.end(); ++s) {
    const Vector2<>& pf = s->pointOnField;
    ColorClasses::Color color = s->isUpperCamera ? ColorClasses::orange : ColorClasses::red;
    int sizeFactor = (theCameraInfo.camera == CameraInfo::upper) ? 2 : 1;
    Vector2<int> pointInImage;
    if (Geometry::calculatePointInImage(Vector3<>(pf.x, pf.y, 0.f), cameraMatrix, theCameraInfo, pointInImage)) {
      CROSS("module:CameraCalibratorV4:drawSamples",
            pointInImage.x,
            pointInImage.y,
            5 * sizeFactor,
            0.5 * sizeFactor,
            Drawings::ps_solid,
            color);
    }
  }
}

MAKE_MODULE(CameraCalibratorV4, Cognition Infrastructure)
